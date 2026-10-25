package org.firstinspires.ftc.teamcode.juniper.wrappers;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ELCv2 extends PosEncoder {
//TODO: fix all the absolute angle stuff (is it needed??)
    private AnalogInput encoder;
    private double lastCheckedPosition, fullRotations;
    private Double currentAbsoluteAngle;


    public ELCv2 (String name, HardwareMap map) {
        super (name, false, map);
        encoder = map.get(AnalogInput.class, name);
        lastCheckedPosition = (encoder.getVoltage() / 3.3) * 360;
        currentAbsoluteAngle = lastCheckedPosition;
    }

    @Override
    public void calculateValue() {
        double encoderResult = (encoder.getVoltage() / 3.3) * 360;

        if (currentAbsoluteAngle != null) {
            currentAbsoluteAngle = encoderResult; // + fullRotations * 360;
        }

        lastCheckedPosition = encoderResult;
    }

    @Override
    public Double getValue() {
        return lastCheckedPosition;
    }

    @NonNull
    @Override
    public String toString() {
        return "current absolute angle " + currentAbsoluteAngle + "\n" +
                "current relative position " + lastCheckedPosition + "\n" +
                "full rotations " + fullRotations  + "\n"
                ;
    }

    @Override
    public double getPosition() {
        return currentAbsoluteAngle;
    }
}
