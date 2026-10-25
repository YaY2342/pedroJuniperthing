package org.firstinspires.ftc.teamcode.juniper.wrappers;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class nEncoder<T> extends Hardware {

    //TODO: i2c differentiation for hwq loading

    private double ups; // update per second

    private double lastCheckedTime;
    private boolean isI2C;

    public nEncoder (String name, double basePrio, double scale, boolean isI2C, HardwareMap map) {
        super(name, basePrio, scale, map);
        lastCheckedTime = System.nanoTime();
        this.isI2C = isI2C;
    }

    public nEncoder(String name, boolean isI2C, HardwareMap map) {
        this(name, 0, 1, isI2C, map);
    }

    @Override
    public void update() {
        calculateValue();
        lastCheckedTime = System.nanoTime();
    }

    public abstract void calculateValue();

    public abstract T getValue();

    public void setRate (double updatesPerSecond) {
        this.ups = updatesPerSecond;
    }
    public double getRate () {
        return ups;
    }

    public boolean isI2C() {
        return isI2C;
    }
    @Override
    public double calculatePriority() {
        if (!isI2C || System.nanoTime() - lastCheckedTime > (1.0E9 / ups)) {
            this.currentPrio = Double.MAX_VALUE;
            return Double.MAX_VALUE;
        }

        this.currentPrio = 0;
        return 0;
    }

    @NonNull
    @Override
    public abstract String toString();
}
