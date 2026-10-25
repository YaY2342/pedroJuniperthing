package org.firstinspires.ftc.teamcode.juniper.util;

public class Utils {
    public static double angleWrap (double radians) {
        radians %= (2 * Math.PI);

        if (radians > Math.PI) {
            radians -= 2 * Math.PI;
        } else if (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        return radians;
    }

    public static double minMaxClip (double val, double min, double max) {
        return Math.max(min, Math.min(val, max));
    }

    public static boolean withinRange (double val, double min, double max, boolean inclusive) {
        if (inclusive) {
            return (val <= max) && (val >= min);
        }
        return (val < max) && (val > min);
    }

    /**
     *
     * @param ticksPerSecond - motor.getVelocity() result
     * @param gearRatio - the input teeth / the output teeth (ie 24:16 would be 1.5)
     * @return output velocity
     */
    public static double getRPM (double ticksPerSecond, double gearRatio) {
        return (ticksPerSecond * (60.0/28) * gearRatio);
    }
}
