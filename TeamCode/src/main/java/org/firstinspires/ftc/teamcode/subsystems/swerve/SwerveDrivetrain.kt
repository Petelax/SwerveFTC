package org.firstinspires.ftc.teamcode.subsystems.swerve

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
    private var imu: IMU

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
        imu = hardwareMap.get(IMU::class.java, "imu")
    }

    fun getHeading(): Double {
        return imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)
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

}