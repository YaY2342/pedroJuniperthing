package org.firstinspires.ftc.teamcode.juniper.wrappers;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class REV3<T> extends nEncoder<T> {
    protected RevColorSensorV3 input;

    public REV3(String name, HardwareMap map) {
        super(name, true, map);
        input = map.get(RevColorSensorV3.class, name);
        setRate(30);
    }
}
