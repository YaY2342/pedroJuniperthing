package org.firstinspires.ftc.teamcode.juniper.wrappers;

import android.annotation.SuppressLint;

import java.util.ArrayList;
import java.util.Collections;

public class HardwareQueue {
    private ArrayList<Hardware> hw = new ArrayList<>();
    public double targetLoopLength = 0.020; //20 ms, write in seconds
    private static long LOOP_START = System.nanoTime();

    public void START_LOOP() {
        LOOP_START = System.nanoTime();
    }
    public double GET_LOOP_TIME() {
        return (System.nanoTime() - LOOP_START) / 1.0e9; // converts from nano secs to secs
    }
    
    public void addDevice(Hardware device) {
        hw.add(device);
    }

    public void update() {
        for (Hardware h : hw) {
            h.calculatePriority();
        }

        hw.sort(Collections.reverseOrder());
        double looptime = GET_LOOP_TIME();
        int index = 0;
        do {
            hw.get(index).update();
            looptime = GET_LOOP_TIME();
            index++;
        } while (looptime <= targetLoopLength && index < hw.size());
    }

    @SuppressLint("DefaultLocale")
    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        double currentLoopTime = GET_LOOP_TIME();

        sb.append("HardwareQueue Status:\n");
        sb.append(String.format("  Target Loop Length: %.3f ms\n", targetLoopLength * 1000));
        sb.append(String.format("  Current Loop Time:  %.3f ms\n", currentLoopTime * 1000));
        sb.append(String.format("  Load:               %.1f%%\n", (currentLoopTime / targetLoopLength) * 100));
        sb.append(String.format("  Device Count:       %d\n", hw.size()));
        sb.append("  Devices (Sorted by Priority):\n");

        if (hw.isEmpty()) {
            sb.append("    [None]");
        } else {
            for (int i = 0; i < hw.size(); i++) {
                Hardware device = hw.get(i);
                // Assumes Hardware class has a meaningful toString() or getName()
                sb.append(String.format("    %d. %s\n", i + 1, device.toString()));
            }
        }

        return sb.toString();
    }
}
