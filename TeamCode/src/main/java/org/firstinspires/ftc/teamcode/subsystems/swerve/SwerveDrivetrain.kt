package org.firstinspires.ftc.teamcode.subsystems.swerve

import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState
import com.outoftheboxrobotics.photoncore.PeriodicSupplier
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import org.ejml.simple.SimpleMatrix
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.SparkFunOTOS
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.pow
import kotlin.math.sin

@Photon
class SwerveDrivetrain {
    private var lf: SwerveModule
    private var rf: SwerveModule
    private var lr: SwerveModule
    private var rr: SwerveModule
    private var kinematics: SwerveDriveKinematics = SwerveDriveKinematics(
        DrivebaseConstants.Measurements.LF_POS,
        DrivebaseConstants.Measurements.RF_POS,
        DrivebaseConstants.Measurements.LR_POS,
        DrivebaseConstants.Measurements.RR_POS
    )
    //private var imu: IMU
    private var odo: SparkFunOTOS

    /* Define the offsets from the center of the robot to each wheel */
    private val r = arrayOf(
        arrayOf(0.13335, 0.13335), // LF
        arrayOf(0.13335, -0.13335), // RF
        arrayOf(-0.13335, 0.13335), // LR
        arrayOf(-0.13335, -0.13335) // RR
    )

    constructor(hardwareMap: HardwareMap) {
        val id = DrivebaseConstants.DeviceIDs
        lf = SwerveModule(hardwareMap, id.LF_DRIVE_MOTOR, id.LF_TURN_MOTOR, id.LF_ENCODER, DrivebaseConstants.Measurements.LF_OFFSET)
        rf = SwerveModule(hardwareMap, id.RF_DRIVE_MOTOR, id.RF_TURN_MOTOR, id.RF_ENCODER, DrivebaseConstants.Measurements.RF_OFFSET)
        lr = SwerveModule(hardwareMap, id.LR_DRIVE_MOTOR, id.LR_TURN_MOTOR, id.LR_ENCODER, DrivebaseConstants.Measurements.LR_OFFSET)
        rr = SwerveModule(hardwareMap, id.RR_DRIVE_MOTOR, id.RR_TURN_MOTOR, id.RR_ENCODER, DrivebaseConstants.Measurements.RR_OFFSET)
        //imu = hardwareMap.get(IMU::class.java, "imu")
        odo = hardwareMap.get(SparkFunOTOS::class.java, "otos")
        configureOtos()
    }

    fun getHeading(): Double {
        return getPose().rotation.radians
        //return imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)
    }

    fun getPose(): Pose2d {
        val pose = odo.position
        return Pose2d(pose.x, pose.y, Rotation2d.fromDegrees(pose.h))
    }


    fun drive(speeds: ChassisSpeeds) {
        //if (hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) >= 0.01 || speeds.omegaRadiansPerSecond >= 0.01) {
        setModuleStates(kinematics.toSwerveModuleStates(speeds))
        //}
    }

    fun firstOrderDrive(speeds: ChassisSpeeds) {
        val moduleStates = firstOrderInverse(speeds)
        setModuleStates(moduleStates)
    }

    fun firstOrderInverse(speeds: ChassisSpeeds): Array<SwerveModuleState> {
        /* Define the robot velocities */
        val vx = speeds.vxMetersPerSecond
        val vy = speeds.vyMetersPerSecond
        val vw = speeds.omegaRadiansPerSecond

        /* Define array to store module states */
        var moduleStates: Array<SwerveModuleState> = Array(4) { SwerveModuleState() }


        for (i in moduleStates.indices) {
            val vxModule = vx + vw * r[i][1]
            val vyModule = vy + vw * r[i][0]
            val moduleVelocity = hypot(vxModule, vyModule)
            val moduleHeading = atan2(vyModule, vxModule)

            moduleStates[i] = SwerveModuleState(moduleVelocity, Rotation2d(moduleHeading))

        }

        return moduleStates
    }

    fun secondOrderInverse(speeds: ChassisSpeeds, accels: ChassisSpeeds): Array<SwerveModule.SwerveModuleStateAccel> {
        var moduleStates: Array<SwerveModule.SwerveModuleStateAccel> = Array(4) { SwerveModule.SwerveModuleStateAccel() }
        for (i in moduleStates.indices) {
            val m = SimpleMatrix(
                arrayOf(
                    doubleArrayOf(1.0, 0.0, -r[i][0], -r[i][1]),
                    doubleArrayOf(0.0, 1.0, -r[i][1], r[i][0])
                )
            )
            val input = SimpleMatrix(
                arrayOf(
                    doubleArrayOf(
                        accels.vxMetersPerSecond, accels.vyMetersPerSecond,
                        speeds.omegaRadiansPerSecond.pow(2.0), accels.omegaRadiansPerSecond
                    )
                )
            )
            val moduleAccels = m.mult(input)

            val vxModule = speeds.vxMetersPerSecond + speeds.omegaRadiansPerSecond * r[0][1]
            val vyModule = speeds.vyMetersPerSecond + speeds.omegaRadiansPerSecond * r[0][0]
            val moduleVelocity = hypot(vxModule, vyModule)
            val moduleHeading = atan2(vyModule, vxModule)

            val m2 = SimpleMatrix(
                arrayOf(
                    doubleArrayOf(cos(moduleHeading), sin(moduleHeading)),
                    doubleArrayOf(-sin(moduleHeading), cos(moduleHeading))
                )
            )
            val final = m2.mult(moduleAccels)

            moduleStates[i] = SwerveModule.SwerveModuleStateAccel(moduleVelocity, moduleHeading, final[0], final[1])
        }

        return moduleStates
    }

    fun secondOrderDrive(speeds: ChassisSpeeds, accels: ChassisSpeeds) {
        val moduleStates = secondOrderInverse(speeds, accels)
        setModuleStatesAccel(moduleStates)
    }


    fun fieldCentricDrive(speeds: ChassisSpeeds) {
        drive(ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond,
            Rotation2d(getHeading())))
    }

    fun firstOrderFieldCentricDrive(speeds: ChassisSpeeds) {
        firstOrderDrive(ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond,
            Rotation2d(getHeading())))
    }

    fun setModuleStates(moduleStates: Array<SwerveModuleState>) {
        SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, DrivebaseConstants.Measurements.MAX_VELOCITY)
        lf.setDesiredState(moduleStates[0])
        rf.setDesiredState(moduleStates[1])
        lr.setDesiredState(moduleStates[2])
        rr.setDesiredState(moduleStates[3])
    }

    fun setModuleStatesAccel(moduleStates: Array<SwerveModule.SwerveModuleStateAccel>) {
        // SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, DrivebaseConstants.Measurements.MAX_VELOCITY)
        lf.setDesiredStateAccel(moduleStates[0])
        rf.setDesiredStateAccel(moduleStates[1])
        lr.setDesiredStateAccel(moduleStates[2])
        rr.setDesiredStateAccel(moduleStates[3])
    }

    fun getModuleHeadings(): Array<Double> {
        return arrayOf(lf.getHeading(), rf.getHeading(), lr.getHeading(), rr.getHeading())
    }

    fun getDesiredModuleStates(): Array<SwerveModuleState> {
        return arrayOf(lf.getDesiredState(), rf.getDesiredState(), lr.getDesiredState(), rr.getDesiredState())

    }

    fun getAThing(): Array<Double> {
        return arrayOf(lf.getThing(), rf.getThing(), lr.getThing(), rr.getThing())
    }

    fun test(drive: Double, steer: Double) {
        lf.spin(drive, steer)
        rf.spin(drive, steer)
        lr.spin(drive, steer)
        rr.spin(drive, steer)
    }

    private fun configureOtos() {
        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // stored in the sensor, it's part of the library, so you need to set at the
        // start of all your programs.
        // otos.setLinearUnit(SparkFunOTOS.LinearUnit.METERS);
        odo.setLinearUnit(SparkFunOTOS.LinearUnit.INCHES)
        // otos.setAngularUnit(SparkFunOTOS.AngularUnit.RADIANS);
        odo.setAngularUnit(SparkFunOTOS.AngularUnit.DEGREES)

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        val offset = SparkFunOTOS.Pose2D(8.0, 0.0, 90.0)
        odo.setOffset(offset)

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        odo.setLinearScalar(1.011)
        odo.setAngularScalar(0.990)

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your programs. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        odo.calibrateImu()

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        odo.resetTracking()

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        val currentPosition = SparkFunOTOS.Pose2D(0.0, 0.0, 0.0)
        odo.setPosition(currentPosition)

        // Get the hardware and firmware version
        val hwVersion = SparkFunOTOS.Version()
        val fwVersion = SparkFunOTOS.Version()
        odo.getVersionInfo(hwVersion, fwVersion)
    }

}