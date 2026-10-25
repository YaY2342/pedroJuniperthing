package org.firstinspires.ftc.teamcode.juniper.wrappers;

import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class Hardware implements Comparable<Hardware> {

    protected final double basePriority, priorityScale;
    public final String name;
    protected double lastUpdateTime, currentPrio;
    public final HardwareMap map;
    boolean forceUpdate = false;

    public Hardware (String name, double basePriority, double priorityScale, HardwareMap map) {
        this.basePriority = basePriority;
        this.priorityScale = priorityScale;
        this.name = name;
        lastUpdateTime = System.nanoTime();
        currentPrio = basePriority;
        this.map = map;
    }

    protected abstract double calculatePriority();

    public abstract void update();

    public void requestForce() {
        forceUpdate = true;
    }

    public String getName() {
        return name;
    }

    public double getCurrentPrio() {
        return currentPrio;
    }

    @Override
    public int compareTo (Hardware other) {
        return (int) (getCurrentPrio() - other.getCurrentPrio());
    }
    public abstract String toString();
}
