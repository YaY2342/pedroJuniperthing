package org.firstinspires.ftc.teamcode.juniper.wrappers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class nMotor extends Hardware {
    private final DcMotorEx motor;
    private double power, velocity, targetPower;
    private boolean collectingVoltage = false;
    private long lastLoop = System.nanoTime();
    private CurrentUnit unit;

    public nMotor (String name, HardwareMap m) {
        this(name, 0, 1, m);
    }

    public nMotor (String name, double basePrio, double prioScale, HardwareMap m) {
        super(name, basePrio, prioScale, m);
        motor = m.get(DcMotorEx.class, name);
        power = 0.0;
        velocity = 0.0;
    }

    @Override
    public void update() {
        motor.setPower(targetPower);
        power = targetPower;
        lastLoop = System.nanoTime();
    }


    @Override
    public double calculatePriority() {
        double delta = Math.abs(targetPower - power);
        if (delta == 0 ) {
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
        if (true) { // need to figure out tolerancing without breaking ts
            targetPower = newP;
        }
    }

    public double getPower() {
        return power;
    }

    public double getVelocity() {
        velocity = motor.getVelocity();
        return velocity;
    }

    public void setCurrentUnit(CurrentUnit unit) {
        this.unit = unit;
    }

    public double getCurrent() {
        collectingVoltage = true;
        return motor.getCurrent((unit != null) ? unit : CurrentUnit.MILLIAMPS);
    }

    public void setCurrentAlert(double amount) {
        collectingVoltage = true;
        motor.setCurrentAlert(amount, (unit != null) ? unit : CurrentUnit.MILLIAMPS);
    }

    public boolean aboveCurrentAlert() {
        return motor.isOverCurrent();
    }

    public double getPosition() {
        return motor.getCurrentPosition();
    }

    public nMotor setDirection(DcMotorSimple.Direction direction) {
        motor.setDirection(direction);
        return this;
    }

    public nMotor setZPB (DcMotor.ZeroPowerBehavior zpb) {
        motor.setZeroPowerBehavior(zpb);
        return this;
    }

    @Override
    public String toString() {
        return "Motor " + name + ": \n" +
                "target power " + targetPower + "\n" +
                "power " + power + "\n" +
                "collecting voltage " + collectingVoltage + "\n" +
                "cached velocity " + velocity + "\n";
    }
}
