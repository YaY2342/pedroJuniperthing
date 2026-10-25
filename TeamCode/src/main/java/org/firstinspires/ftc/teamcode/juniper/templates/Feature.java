package org.firstinspires.ftc.teamcode.juniper.templates;

import com.example.juniper.wrappers.Hardware;

import java.util.ArrayList;

public abstract class Feature {
    public final ArrayList<Hardware> hw;
    public final String name;
    public boolean inHWQ = false;

    protected Feature(String name, ArrayList<Hardware> hw) {
        this.name = name;
        this.hw = hw;
    }

    public void update() {
        if (!inHWQ) {
            for (Hardware h : hw) {
                h.update();
            }
        }
        updateCall();
    }

    protected abstract void updateCall();

    @Override
    public String toString() {
        StringBuilder toReturn = new StringBuilder(name + " {\n");
        for (Hardware h : hw) {
            toReturn.append(h.toString()).append(", \n");
        }
        return toReturn + "}";
    }
}
