package org.firstinspires.ftc.teamcode.subsystems.swerve

import com.arcrobotics.ftclib.geometry.Translation2d
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants

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

    constructor(hardwareMap: HardwareMap) {
        val id = DrivebaseConstants.DeviceIDs
        lf = SwerveModule(hardwareMap, id.LF_DRIVE_MOTOR, id.LF_TURN_MOTOR, id.LF_ENCODER)
        rf = SwerveModule(hardwareMap, id.RF_DRIVE_MOTOR, id.RF_TURN_MOTOR, id.RF_ENCODER)
        lr = SwerveModule(hardwareMap, id.LR_DRIVE_MOTOR, id.LR_TURN_MOTOR, id.LR_ENCODER)
        rr = SwerveModule(hardwareMap, id.RR_DRIVE_MOTOR, id.RR_TURN_MOTOR, id.RR_ENCODER)
    }



}