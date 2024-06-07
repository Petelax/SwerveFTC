package org.firstinspires.ftc.teamcode.subsystems.swerve

import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState
import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.subsystems.AbsoluteAnalogEncoder
import org.firstinspires.ftc.teamcode.utils.PIDController
import kotlin.math.abs
import kotlin.math.sign

class SwerveModule {
    private var motor: DcMotorEx
    private var servo: CRServoImplEx
    private var encoder: AbsoluteAnalogEncoder
    private var desiredState: SwerveModuleState
    private var driveFeedForward: SimpleMotorFeedforward
    private var turnPID: PIDController

    /**
     * @param hardwareMap hardwareMap
     * @param m drive motor id
     * @param s steer servo id
     * @param e analog encoder id
     * @param encoderOffset offset for analog encoder
     */
    constructor(hardwareMap: HardwareMap, m: String, s: String, e: String, encoderOffset: Double) {
        this.motor = hardwareMap.get(DcMotorEx::class.java, m)
        this.servo = hardwareMap.get(CRServoImplEx::class.java, s)
        this.encoder = AbsoluteAnalogEncoder(hardwareMap, e, encoderOffset)
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
        servo.direction = DcMotorSimple.Direction.REVERSE

        turnPID.enableContinuousInput(0.0, 2.0*Math.PI)
    }

    fun getHeading(): Double {
        return encoder.getHeading()
    }

    fun getDesiredState(): SwerveModuleState {
        return desiredState
    }

    fun setDesiredState(state: SwerveModuleState) {
        desiredState = SwerveModuleState.optimize(state, Rotation2d(getHeading()))

        val drivePower = driveFeedForward.calculate(desiredState.speedMetersPerSecond) / 12.0
        var turnPower = turnPID.calculate(getHeading(), state.angle.radians)

        //(Math.abs(error) > 0.02 ? K_STATIC : 0) * Math.signum(power)
        turnPower += if (abs(turnPID.positionError) > 0.02) 0.03 else 0.0 * sign(turnPower)

        servo.power = turnPower
        motor.power = drivePower
    }

    /**
     * Spins modules
     * @param drive drive motor power. -1 to 1
     * @param steer steer motor power. -1 to 1
     */
    fun spin(drive: Double, steer: Double) {
        motor.power = drive
        servo.power = steer
    }

}