package org.firstinspires.ftc.teamcode.juniper.templates;

import java.util.ArrayList;

public abstract class Subsystem {
    public final ArrayList<Feature> features;
    public final String name;

    protected Subsystem(String name, ArrayList<Feature> features) {
        this.name = name;
        this.features = features;
    }

    public void update() {
        for (Feature f : features) {
            f.update();
        }
        updateCall();
    }

    protected abstract void updateCall();

    @Override
    public String toString() {
        StringBuilder toReturn = new StringBuilder(name + " {\n");
        for (Feature f : features) {
            toReturn.append(f.toString()).append(", \n");
        }
        return toReturn + "}";
    }
}
