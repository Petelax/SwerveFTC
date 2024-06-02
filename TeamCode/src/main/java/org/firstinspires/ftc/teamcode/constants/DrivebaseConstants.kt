package org.firstinspires.ftc.teamcode.constants

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

    object ModuleCoefficients {
        const val KS = 0.0
        const val KV = 0.0
        const val KA = 0.0

        const val KP = 0.01
        const val KI = 0.0
        const val KD = 0.0
    }

    object Measurements {
        const val PI = 3.1415926535897932384626433832795028841971693993751058209
        const val TRACK_WIDTH = 10.50
        const val WHEEL_BASE = 10.50
        const val WHEEL_RADIUS = 2.835/2.0
        const val TICKS_PER_REV = 8192
        const val TICKS_TO_INCHES = (PI*2.0*WHEEL_RADIUS) / TICKS_PER_REV
        const val MAX_VELOCITY = 3.8 /* meters per second */
        const val MAX_ACCEL = 15.0 /* meters per second */
        const val MAX_ANGULAR_VELOCITY = 16.0
        const val CENTER_WHEEL_OFFSET = 0.0
        private const val k = TRACK_WIDTH*0.0254/2
        private const val j = WHEEL_BASE*0.0254/2

        const val MAX_VELOCITY_IN = 62.0 /* meters per second */
        const val MAX_ANGULAR_VELOCITY_RAD = 10

        const val MAX_ELEVATOR_HEIGHT = 2190
        const val ELEVATOR_TICKS_PER_INCH = 81.1111
        const val ELEVATOR_PPR = 384.5

        val LF_POS = Translation2d(k/2, j/2)
        val RF_POS = Translation2d(k/2, -j/2)
        val LR_POS = Translation2d(-k/2, j/2)
        val RR_POS = Translation2d(-k/2, -j/2)

        const val LF_OFFSET = -1.175
        const val RF_OFFSET = -0.23
        const val LR_OFFSET = -0.98
        const val RR_OFFSET = -1.85


    }

}