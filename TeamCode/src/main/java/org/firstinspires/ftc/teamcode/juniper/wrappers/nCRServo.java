package org.firstinspires.ftc.teamcode.juniper.wrappers;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class nCRServo extends Hardware {

    private CRServo servo;
    private PosEncoder encoder;
    private double power, targetPower;
    private long lastLoop = System.nanoTime();


    public nCRServo (String name, double basePrio, double prioScale, PosEncoder encoder, HardwareMap m) {
        super(name, basePrio, prioScale, m);
        servo = m.get(CRServo.class, name);
        this.encoder = encoder;
    }

    public nCRServo (String name, PosEncoder encoder, HardwareMap m) {
        this(name, 0, 1, encoder, m);
    }

    @Override
    public void update() {
        servo.setPower(targetPower);
        power = targetPower;

        if (encoder != null) {
            encoder.update();
        }

        lastLoop = System.nanoTime();
    }
    
    @Override
    public double calculatePriority() {
        double delta = Math.abs(power - targetPower);
        if (delta == 0) {
            currentPrio = 0;
            return currentPrio;
        }

        double prio = basePriority;

        long currentTime = System.nanoTime();
        double timeSinceLast = ((double) currentTime - lastLoop) / 1.0E9;
        prio += (timeSinceLast * 0.5);

        prio += delta * 100;

        if (forceUpdate) {
            prio += 100000;
            forceUpdate = false;
        }

        this.currentPrio = prio * priorityScale;
        return currentPrio;
    }

    public void setPower(double newP) {
        targetPower = newP;
    }

    public double getPower() {
        return power;
    }

    public PosEncoder getEncoder() {
        if (encoder == null) throw new UnsupportedOperationException("ENCODER IS NULL");
        return encoder;
    }

    public double getEncoderValue() {
        if (encoder == null) throw new UnsupportedOperationException("ENCODER IS NULL");
        return encoder.getValue();
    }

    public void setUpdateRate(double updatesPerSecond) {
        if (encoder == null) return;
        encoder.setRate(updatesPerSecond);
    }

    public double getUpdateRate() {
        if (encoder == null) throw new UnsupportedOperationException("ENCODER IS NULL");
        return encoder.getRate();
    }

    @NonNull
    @Override
    public String toString() {
        return "CRServo " + name + ":\n" +
                "current power " + power + "\n" +
                "target power " + targetPower + "\n" +
                "encoder details " + encoder + "\n";
    }
}
