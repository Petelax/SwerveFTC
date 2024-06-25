package org.firstinspires.ftc.teamcode.constants

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.geometry.Translation2d

class DrivebaseConstants {
    object DeviceIDs {
        const val LF_DRIVE_MOTOR = "motorLF"
        const val LF_TURN_MOTOR = "servoLF"
        const val LF_ENCODER = "encoderLF"

        const val RF_DRIVE_MOTOR = "motorRF"
        const val RF_TURN_MOTOR = "servoRF"
        const val RF_ENCODER = "encoderRF"

        const val LR_DRIVE_MOTOR = "motorLR"
        const val LR_TURN_MOTOR = "servoLR"
        const val LR_ENCODER = "encoderLR"

        const val RR_DRIVE_MOTOR = "motorRR"
        const val RR_TURN_MOTOR = "servoRR"
        const val RR_ENCODER = "encoderRR"

        const val ODOMETRY_POD_PARA = LR_DRIVE_MOTOR // 0
        const val ODOMETRY_POD_PERP = RR_DRIVE_MOTOR // 3

        const val ELEVATOR_LEFT = "elevatorLeft"
        const val ELEVATOR_RIGHT = "elevatorRight"

        const val ARM = "arm"
        const val INTAKE = "intake"

        const val WRIST = "wrist"
        const val CLAW_LEFT = "clawLeft"
        const val CLAW_RIGHT = "clawRight"

        /*
        Control Hub:
            Motor:
                0: LR
                1: LF
                2: RF
                3: RR
            Servo:
                0: LR
                1: LF
                2: RR
                3: RF
            Analog Input:
                0: LR
                1: LF
                2: RR
                3: RF

         Expansion Hub:
            Motor:
                0: elevatorLeft
                1: elevatorRight
                2: arm
                3: intake
            Servo:
                0: clawLeft
                1: clawRight
                2: wrist

        Webcam 1
        */

    }

    @Config
    object ModuleCoefficients {
        @JvmField var KS = 0.0
        @JvmField var KV = 8.0
        @JvmField var KA = 0.0

        @JvmField var KP = 0.5
        @JvmField var KI = 0.0
        @JvmField var KD = 0.0
    }

    object Measurements {
        const val PI = 3.1415926535897932384626433832795028841971693993751058209
        const val INCHES_TO_METERS = 0.0254
        const val TRACK_WIDTH = 10.50 * INCHES_TO_METERS
        const val WHEEL_BASE = 10.50 * INCHES_TO_METERS
        const val WHEEL_RADIUS = 0.072
        const val TICKS_PER_REV = 8192
        //const val TICKS_TO_INCHES = (PI*2.0*WHEEL_RADIUS) / TICKS_PER_REV
        const val MAX_VELOCITY = 1.8 /* meters per second */
        const val MAX_ACCELERATION = 3.6
        //const val MAX_ACCEL = 15.0 /* meters per second */
        const val MAX_ANGULAR_VELOCITY = 10 //13.5 /* rad per second */
        //const val CENTER_WHEEL_OFFSET = 0.0
        private const val k = TRACK_WIDTH/2
        private const val j = WHEEL_BASE/2

        //const val MAX_ANGULAR_VELOCITY_RAD = 10

        val LF_POS = Translation2d(k, j)
        val RF_POS = Translation2d(k, -j)
        val LR_POS = Translation2d(-k, j)
        val RR_POS = Translation2d(-k, -j)

        const val LF_OFFSET = 1.983
        const val RF_OFFSET = 0.297
        const val LR_OFFSET = 3.337
        const val RR_OFFSET = 2.055


    }

}