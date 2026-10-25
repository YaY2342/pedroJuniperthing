package org.firstinspires.ftc.teamcode.juniper.util;

public class BetterTimer {

    private long startTimeNs;

    public BetterTimer() {
        resetTimer();
    }

    public void resetTimer() {
        startTimeNs = System.nanoTime();
    }

    public long getElapsedTime() {
        return (System.nanoTime() - startTimeNs) / 1_000_000L;
    }

    public long getElapsedTimeNs() {
        return (System.nanoTime() - startTimeNs);

    }

    public double getElapsedTimeSeconds() {
        return (System.nanoTime() - startTimeNs) * 1e-9;
    }
}