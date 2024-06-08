package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareMap

class AbsoluteAnalogEncoder {
    private var encoder: AnalogInput
    private var offset = 0.0
    private val range = 3.3
    constructor(hardwareMap: HardwareMap, name: String, offset: Double) {
        this.encoder = hardwareMap.get(AnalogInput::class.java, name)
        this.offset = offset
    }

    constructor(encoder: AnalogInput) {
        this.encoder = encoder
    }


    fun getHeading(): Double {
        return ((((getVoltage()) / range) * Math.PI*2) - offset).mod(2*Math.PI)
    }

    fun getVoltage(): Double {
        return encoder.voltage
    }

}