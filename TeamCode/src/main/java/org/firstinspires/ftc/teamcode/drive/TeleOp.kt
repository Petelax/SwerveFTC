package org.firstinspires.ftc.teamcode.drive

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveDrivetrain

@Photon
@TeleOp
class TeleOp: OpMode() {
    private lateinit var hubs: List<LynxModule>
    private lateinit var elapsedtime: ElapsedTime
    private lateinit var drive: SwerveDrivetrain
    private lateinit var gamepad: GamepadEx

    override fun init() {
        elapsedtime = ElapsedTime()
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        // this just sets the bulk reading mode for each hub
        hubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in hubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }

        drive = SwerveDrivetrain(hardwareMap)
        gamepad = GamepadEx(gamepad1)

        elapsedtime.reset()
    }

    override fun loop() {
        // clears the cache on each hub
        for (hub in hubs) {
            hub.clearBulkCache()
        }

        drive.drive(ChassisSpeeds(
            gamepad.leftX*DrivebaseConstants.Measurements.MAX_VELOCITY,
            gamepad.leftY*DrivebaseConstants.Measurements.MAX_VELOCITY,
            gamepad.rightX*DrivebaseConstants.Measurements.MAX_ANGULAR_VELOCITY
        ))

        //drive.test(gamepad.leftY, gamepad.rightX)
        val headings = drive.getModuleHeadings()
        val states = drive.getDesiredModuleStates()
        val gyro = drive.getHeading()
        telemetry.addData("thing", drive.getAThing()[0])
        telemetry.addData("heading", gyro)
        telemetry.addData("lf heading", headings[0])
        telemetry.addData("lf desired heading", states[0].angle.radians)
        telemetry.addData("rf heading", headings[1])
        telemetry.addData("rf desired heading", states[1].angle.radians)
        telemetry.addData("lr heading", headings[2])
        telemetry.addData("lr desired heading", states[2].angle.radians)
        telemetry.addData("rr heading", headings[3])
        telemetry.addData("rr desired heading", states[3].angle.radians)

        telemetry.addData("ms", elapsedtime.milliseconds())
        elapsedtime.reset()
    }}