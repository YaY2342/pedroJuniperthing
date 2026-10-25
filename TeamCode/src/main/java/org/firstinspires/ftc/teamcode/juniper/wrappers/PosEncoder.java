package org.firstinspires.ftc.teamcode.juniper.wrappers;

import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class PosEncoder extends nEncoder<Double> {
    public PosEncoder(String name, boolean isI2C, HardwareMap map) {
        super(name, isI2C, map);
    }

    public abstract double getPosition();
}
