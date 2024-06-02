package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareMap

class AbsoluteAnalogEncoder {
    private var encoder: AnalogInput
    private var offset = 0
    private val range = 3.3
    constructor(hardwareMap: HardwareMap, name: String) {
        this.encoder = hardwareMap.get(AnalogInput::class.java, name)
    }

    constructor(encoder: AnalogInput) {
        this.encoder = encoder
    }


    public fun getHeading(): Double {
        return (((getVoltage()/range) * Math.PI*2) - offset)%(2*Math.PI)
    }

    public fun getVoltage(): Double {
        return encoder.voltage
    }

}