package org.firstinspires.ftc.teamcode.juniper.templates;

import com.example.juniper.util.BulkReads;
import com.example.juniper.wrappers.Hardware;
import com.example.juniper.wrappers.HardwareQueue;
import com.example.juniper.wrappers.nEncoder;

import java.util.ArrayList;

public abstract class Robot {
    private HardwareQueue hwq;
    private ArrayList<Subsystem> systems;
    private ArrayList<nEncoder> nonI2C;
    private BulkReads bulkReads;
    private String name;

    protected Robot (String name) {
        this.name = name;
        this.systems = new ArrayList<>();
        this.hwq = new HardwareQueue();
    }

    public void addSystem (Subsystem ss) {
        systems.add(ss);
    }

    public void init() {
        nonI2C = new ArrayList<>();

        Hardware lastAdded = null;

        for (Subsystem s : systems) {
            for (Feature f : s.features) {
                f.inHWQ = true;
                for (Hardware hw : f.hw) {
                    if (!(hw instanceof nEncoder) || ((nEncoder<?>) hw).isI2C()) {
                        hwq.addDevice(hw);
                    } else {
                        nonI2C.add((nEncoder) hw);
                    }
                    lastAdded = hw;
                }
            }
        }

        assert lastAdded != null;
        bulkReads = new BulkReads(lastAdded.map);
    }

    public void setTargetLoopTimes (int ms) {
        hwq.targetLoopLength = 0.001 * ms;
    }
    public void setTargetLoopTimes (double s) {
        hwq.targetLoopLength = s;
    }

    public void update() {
        hwq.START_LOOP();
        bulkReads.updateReads();

        for (nEncoder e : nonI2C) {
            e.update();
        }

        for (Subsystem s : systems) {
            s.update();
        }

        hwq.update();
    }

    @Override
    public String toString() {
        StringBuilder toReturn = new StringBuilder(name + " {\n");
        toReturn.append(hwq.toString()).append(", \n");
        for (Subsystem s : systems) {
            toReturn.append(s.toString()).append(", \n");
        }
        return toReturn + "}";
    }
}
