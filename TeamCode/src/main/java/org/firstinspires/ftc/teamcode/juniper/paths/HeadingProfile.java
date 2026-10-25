package org.firstinspires.ftc.teamcode.juniper.paths;

import com.example.juniper.util.Utils;

public abstract class HeadingProfile {
    public abstract double degrees (double s, double len);
    public abstract double ddt (double s);

    public static class Tangential extends HeadingProfile {
        private final CompleteSpline spline;
        public Tangential (CompleteSpline spline) {
            this.spline = spline;
        }

        @Override
        public double degrees(double s, double len) {
            return spline.pathDir(s);
        }

        @Override
        public double ddt (double s) {
            return spline.curvature(s);
        }
    }
    public static class Constant extends HeadingProfile {
        private final double heading; // radians
        public Constant (double degrees) {
            this.heading = Math.toRadians(degrees);
        }

        @Override
        public double degrees(double s, double len) {
            return heading;
        }

        @Override
        public double ddt (double s) {
            return 0;
        }
    }
    public static class Linear extends HeadingProfile {
        private final double startRad, endRad, delta;
        private double len;

        public Linear(double startDeg, double endDeg) {
            this.startRad = Math.toRadians(startDeg);
            this.endRad = Math.toRadians(endDeg);
            this.delta = Utils.angleWrap(endRad - startRad);
        }

        @Override
        public double degrees(double s, double len) {
            double t = s / len;

            return startRad + t * delta;
        }

        @Override
        public double ddt (double s) {
            return Math.toDegrees(delta) / len;
        }

        public double ddt (double s, double len) {
            setLen(len);
            return ddt(s);
        }

        public void setLen (double len) {
            this.len = len;
        }
        public double rate () {
            return delta;
        }
    }
}
