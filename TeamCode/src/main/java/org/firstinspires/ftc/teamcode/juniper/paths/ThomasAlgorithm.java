package org.firstinspires.ftc.teamcode.juniper.paths;

public class ThomasAlgorithm {
    public static double[] solve(double[] a, double[] b, double[] c, double[] d) {
        // assume all arrays are same length - throw errors
        // throw error if a[0] is not 0, if c[n] is not 0

        double[][] cd = forwardSweep(a, b, c, d);
        return backwardSweep(cd[0], cd[1]);
    }

    private static double[][] forwardSweep (double[] a, double[] b, double[] c, double[] d) {
        double[][] values = new double[2][c.length];

        int i = 0, n = d.length - 1;

        values[0][0] = c[0] / b[0];
        values[1][0] = d[0] / b[0];

        for (i = 1; i < n; i++) {
            values[0][i] = c[i] / (b[i] - (a[i] * values[0][i - 1]));
            values[1][i] = (d[i] - (a[i] * values[1][i - 1])) / (b[i] - (a[i] * values[0][i - 1]));
        }

        values[1][n] = (d[i] - (a[i] * values[1][i - 1])) / (b[i] - (a[i] * values[0][i - 1]));

        return values;
    }

    private static double[] backwardSweep (double[] c, double[] d) {
        double[] m = new double[d.length];

        int n = m.length - 1;
        m[n] = d[n];

        for (n--; n >= 0; n--) {
            m[n] = d[n] - (c[n] * m[n+1]);
        }

        return m;
    }
}
