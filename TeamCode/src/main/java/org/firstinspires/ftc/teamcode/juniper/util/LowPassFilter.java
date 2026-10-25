package org.firstinspires.ftc.teamcode.juniper.util;

public class LowPassFilter {
    private final double alpha;
    private double output = 0f;

    // alpha: 0.0 (heavy filtering) to 1.0 (no filtering)
    public LowPassFilter(double alpha) {
        this.alpha = alpha;
    }

    public double filter(double input) {
        if (output == 0) {
            output = input;
            return input;
        }
        output += alpha * (input - output);
        return output;
    }

    public void reset(double value) {
        output = value;
    }

    public void reset() {
        output = 0f;
    }
}