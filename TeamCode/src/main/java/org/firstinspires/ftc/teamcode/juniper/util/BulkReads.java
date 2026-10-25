package org.firstinspires.ftc.teamcode.juniper.util;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class BulkReads {
    private final List<LynxModule> hubs;

    public BulkReads(HardwareMap map) {
        hubs = map.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    /**
     * Should be called once a loop
     */
    public void updateReads() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }

    /**
     * use sparingly
     */
    public void forceRead() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }
}