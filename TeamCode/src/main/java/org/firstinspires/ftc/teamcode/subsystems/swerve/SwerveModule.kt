package org.firstinspires.ftc.teamcode.subsystems.swerve

import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState
import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.subsystems.AbsoluteAnalogEncoder

class SwerveModule {
    private var motor: DcMotorEx
    private var servo: CRServoImplEx
    private var encoder: AbsoluteAnalogEncoder
    private var desiredState: SwerveModuleState
    private var driveFeedForward: SimpleMotorFeedforward
    private var turnPID: PIDController

    constructor(hardwareMap: HardwareMap, m: String, s: String, e: String) {
        this.motor = hardwareMap.get(DcMotorEx::class.java, m)
        this.servo = hardwareMap.get(CRServoImplEx::class.java, s)
        this.encoder = AbsoluteAnalogEncoder(hardwareMap, e)
        this.desiredState = SwerveModuleState()
        val c = DrivebaseConstants.ModuleCoefficients
        this.driveFeedForward = SimpleMotorFeedforward(c.KS, c.KV, c.KA)
        this.turnPID = PIDController(c.KP, c.KI, c.KD)
        initialize()
    }

    constructor(m: DcMotorEx, s: CRServoImplEx, e: AbsoluteAnalogEncoder) {
        this.motor = m
        this.servo = s
        this.encoder = e
        this.desiredState = SwerveModuleState()
        val c = DrivebaseConstants.ModuleCoefficients
        this.driveFeedForward = SimpleMotorFeedforward(c.KS, c.KV, c.KA)
        this.turnPID = PIDController(c.KP, c.KI, c.KD)
        initialize()
    }

    private fun initialize() {
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        servo.pwmRange = PwmControl.PwmRange(500.0, 2500.0, 5000.0)

    }

    public fun getHeading(): Double {
        return encoder.getHeading()
    }

    public fun setDesiredState(state: SwerveModuleState) {
        desiredState = SwerveModuleState.optimize(state, Rotation2d(getHeading()))

        val drivePower = driveFeedForward.calculate(desiredState.speedMetersPerSecond) / 12.0
        val turnPower = turnPID.calculate(getHeading(), state.angle.radians)

        servo.power = turnPower
        motor.power = drivePower
    }





}