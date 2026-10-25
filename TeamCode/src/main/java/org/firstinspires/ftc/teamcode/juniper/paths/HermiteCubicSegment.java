package org.firstinspires.ftc.teamcode.juniper.paths;

import com.example.juniper.util.Utils;

public class HermiteCubicSegment {

    private double a, b, c, d, h;

    public HermiteCubicSegment(double x0, double x1, double m0, double m1, double h) {
        this.d = x0;
        this.c = m0;
        this.b = (3 * (x1 - x0) / (h * h)) - ((2 * m0) + m1) / h;
        this.a = ((m1 + m0) / (h * h)) - (2 * (x1 - x0) / (h * h * h));

        this.h = h;
    }

    public HermiteCubicSegment(double x0, double x1, double m0, double m1) {
        this(x0, x1, m0, m1, 1);
    }

    public double[] getCoeffs() {
        return new double[] {a, b, c, d};
    }


    public double interpolate(double t) {
        t = Utils.minMaxClip(t, 0, h);

        return (a * t * t * t) + (b * t * t) + (c * t) + d;
    }

    public double interpolate(double t, boolean percentage) {
        if (!percentage) return interpolate(t);

        return interpolate(t * h);
    }

    public double firstDerivative (double t) {
        t = Utils.minMaxClip(t, 0, h);

        return (3 * a * t * t) + (2 * b * t) + c;
    }

    public double secondDerivative (double t) {
        t = Utils.minMaxClip(t, 0, h);

        return (6 * a * t) + (2 * b);
    }

    public double getA() {
        return a;
    }
    public double getB() {
        return b;
    }
    public double getC() {
        return c;
    }
    public double getD() {
        return d;
    }
    public double getH() {
        return h;
    }
}
