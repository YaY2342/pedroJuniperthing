package org.firstinspires.ftc.teamcode.juniper.paths;

import com.example.juniper.util.Pose2d;

import java.util.ArrayList;
import java.util.List;

public class CompleteSpline {
    public static class PathState {
        public Pose2d pose;
        public double velocity;
        public double heading, radians, degrees;
        public double curvature;
        public double s, t;

        public PathState (Pose2d pose, double velocity, double degrees, double curvature, double s, double t) {
            this.pose = pose;
            this.velocity = velocity;
            this.heading = degrees;
            this.degrees = degrees;
            this.radians = Math.toRadians(degrees);
            this.curvature = curvature;
            this.s = s;
            this.t = t;
        }
    }
    public static class Builder {
        private final List<Pose2d> poses = new ArrayList<>();
        private double initVel, finalVel;
        private double initH, finalH;
        private boolean modifiedInit, modifiedFinal;
        private HeadingProfile heading;
        private final double vMax, aMax, aLatMax;

        public Builder (double vMax, double aMax, double aLatMax) {
            this.vMax = vMax;
            this.aMax = aMax;
            this.aLatMax = aLatMax;
        }

        public Builder addPose(Pose2d pose) {
            poses.add(pose);
            return this;
        }
        public Builder addPose(double x, double y) {
            poses.add(new Pose2d(x, y));
            return this;
        }

        public Builder initialVelocity(double vel) {
            initVel = vel;
            modifiedInit = true;
            return this;
        }
        public Builder finalVelocity(double vel) {
            finalVel = vel;
            modifiedFinal = true;
            return this;
        }
        public Builder velocityPair(double init, double fin) {
            initialVelocity(init);
            return finalVelocity(fin);
        }

        public Builder initialDirection (double degrees) {
            initH = Math.toRadians(degrees);
            modifiedInit = true;
            return this;
        }
        public Builder finalDirection (double degrees) {
            finalH = Math.toRadians(degrees);
            modifiedFinal = true;
            return this;
        }
        public Builder directionPair (double iDegrees, double fDegrees) {
            initialDirection(iDegrees);
            return finalDirection(fDegrees);
        }

        public Builder setTangentialHeading () {
            heading = null;
            return this;
        }
        public Builder setConstantHeading (double degrees) {
            heading = new HeadingProfile.Constant(degrees);
            return this;
        }
        public Builder setLinearHeading (double startDegrees, double endDegrees) {
            heading = new HeadingProfile.Linear(startDegrees, endDegrees);
            return this;
        }

        public CompleteSpline build () {
            // throw error if <2 points

            CompleteSpline spline;

            double p = 0.0001;
            double mx0 = modifiedInit ? (((initVel == 0) ? p : initVel) * Math.cos(initH)) : 0;
            double my0 = modifiedInit ? (((initVel == 0) ? p : initVel) * Math.sin(initH)) : 0;

            double mx1 = modifiedFinal ? (((finalVel == 0) ? p : finalVel) * Math.cos(finalH)) : 0;
            double my1 = modifiedFinal ? (((finalVel == 0) ? p : finalVel) * Math.sin(finalH)) : 0;

            int n = poses.size() - 1;

            if (n > 1) {
                double[] xPoses = new double[n + 1], yPoses = new double[n + 1], h = new double[n];

                for (int i = 0; i < n + 1; i++) {
                    xPoses[i] = poses.get(i).x;
                    yPoses[i] = poses.get(i).y;

                    if (i > 0) {
                        double dx = xPoses[i] - xPoses[i - 1], dy = yPoses[i] - yPoses[i - 1];

                        h[i - 1] = Math.sqrt(dx * dx + dy * dy);
                    }
                }


                HermiteCubicSegment[] x = generateDimension(n - 1, xPoses, h, mx0, mx1), y = generateDimension(n - 1, yPoses, h, my0, my1);
                SplineSegment[] segments = new SplineSegment[n];

                for (int i = 0; i < x.length; i++) {
                    segments[i] = new SplineSegment(x[i], y[i], h[i]);
                }

                spline =  new CompleteSpline(segments);
            } else {

                double dx = poses.get(1).x - poses.get(0).x, dy = poses.get(1).y - poses.get(0).y;
                double h = Math.sqrt(dx * dx + dy * dy);

                SplineSegment curve = new SplineSegment(
                        new HermiteCubicSegment(
                                poses.get(0).x,
                                poses.get(1).x,
                                mx0,
                                mx1,
                                h
                        ),
                        new HermiteCubicSegment(
                                poses.get(0).y,
                                poses.get(1).y,
                                my0,
                                my1,
                                h
                        ),
                        h
                );
                spline = new CompleteSpline(new SplineSegment[]{curve});
            }

            if (heading == null) {
                spline.setHeading(new HeadingProfile.Tangential(spline));
            } else {
                spline.setHeading(heading);
            }

            spline.setVelocityConstraints(vMax, aMax, aLatMax);
            spline.setPathConstraints(initVel, finalVel);
            spline.computeSamples();

            return spline;
        }

        private HermiteCubicSegment[] generateDimension (int n, double[] p, double[] h, double m0, double m1) {
            double[] a = new double[n], b = new double[n], c = new double[n], d = new double[n];

            // i = 0
            b[0] = 2 * (h[0] + h[1]);
            c[0] = h[0];
            d[0] = 3 * ((h[0] / h[1]) * (p[2] - p[1]) + (h[1] / h[0]) * (p[1] - p[0])) - (m0 * h[1]);

            for (int i = 1; i < n - 1; i++) {
                a[i] = h[i + 1];
                b[i] = 2 * (h[i] + h[i + 1]);
                c[i] = h[i];
                d[i] = 3 * ((h[i] / h[i + 1]) * (p[i + 2] - p[i + 1]) + (h[i + 1] / h[i]) * (p[i + 1] - p[i]));
            }

            // i = n
            a[n - 1] = h[n];
            b[n - 1] = 2 * (h[n - 1] + h[n]);
            d[n - 1] = 3 * ((h[n - 1] / h[n]) * (p[n + 1] - p[n]) + (h[n] / h[n - 1]) * (p[n] - p[n - 1])) - (m1 * h[n - 1]);

            double[] m = ThomasAlgorithm.solve(a, b, c, d);

            HermiteCubicSegment[] segments = new HermiteCubicSegment[n + 1];
            segments[0] = new HermiteCubicSegment(p[0], p[1], m0, m[0], h[0]);
            segments[n] = new HermiteCubicSegment(p[n], p[n + 1], m[n - 1], m1, h[n]);

            for (int i = 1; i < n; i++) {
                segments[i] = new HermiteCubicSegment(p[i], p[i + 1], m[i - 1], m[i], h[i]);
            }

            return segments;
        }
    }
    private static class SplineSegment {
        private final HermiteCubicSegment x, y;
        private final double h;
        private double arcLen;
        private double[] lut;
        private final int SAMPLES = 32;

        public SplineSegment (HermiteCubicSegment x, HermiteCubicSegment y, double h) {
            this.x = x;
            this.y = y;
            this.h = h;

            buildLUT();
        }

        private void buildLUT() {
            lut = new double[SAMPLES + 1];
            lut[0] = 0;
            double accumulated = 0;

            double prevY = y.interpolate(0), prevX = x.interpolate(0);

            for (int i = 1; i < SAMPLES + 1; i++) {
                double t = (h * i) / SAMPLES; //chord parameter

                double curX = x.interpolate(t), curY = y.interpolate(t);
                double dx = curX - prevX, dy = curY - prevY;

                double dist = Math.sqrt((dx * dx) + (dy * dy));
                accumulated += dist;

                lut[i] = accumulated;

                prevX = curX;
                prevY = curY;
            }

            this.arcLen = accumulated;
        }

        /**
         * sample the spline at arc length s
         * @param s [0, arc length]
         * @return pose - x, y of the point at that arc length
         */
        public Pose2d interpolate(double s) {
            double t = getChordLen(s);

            return new Pose2d(x.interpolate(t), y.interpolate(t));
        }

        public double curvature(double s) {
            double t = getChordLen(s);

            double dx = x.firstDerivative(t), dy = y.firstDerivative(t);
            double ddx = x.secondDerivative(t), ddy = y.secondDerivative(t);
            double speed2 = dx * dx + dy * dy;

            return (dx * ddy - dy * ddx) / Math.pow(speed2, 1.5);
        }

        public double heading (double s) {
            double t = getChordLen(s);

            return Math.atan2(y.firstDerivative(t), x.firstDerivative(t));
        }

        public double[] sampleAll (double s) {
            double t = getChordLen(s);

            double x_t = x.interpolate(t), y_t = y.interpolate(t);
            double dx = x.firstDerivative(t), dy = y.firstDerivative(t);
            double ddx = x.secondDerivative(t), ddy = y.secondDerivative(t);

            double speed2 = dx * dx + dy * dy;
            double curvature = (dx * ddy - dy * ddx) / Math.pow(speed2, 1.5);

            double pathDir = Math.atan2(y.firstDerivative(t), x.firstDerivative(t));

            return new double[] {
                    x_t, y_t,
                    pathDir, curvature
            };
        }

        /**
         *
         * @param s [0, arc length]
         * @return t - [0, chord length]
         */
        private double getChordLen (double s) {
            if (s <= 0) return 0;
            if (s >= arcLen) return h;

            //binary search, low <= s < high
            int low = 0, high = SAMPLES, mid;
            while (low < high - 1) {
                mid = (low + high) / 2;
                if (lut[mid] < s) low = mid;
                else high = mid;
            }

            double tHigh = (high * h) / SAMPLES, tLow = (low * h) / SAMPLES;
            double sHigh = lut[high], sLow = lut[low];

            double m = (tHigh - tLow) / (sHigh - sLow); // interpolation

            return tLow + (s - sLow) * m; // weighted interpolation, low + the distance between s and sLow times the slope
        }

        public HermiteCubicSegment x() {
            return x;
        }
        public HermiteCubicSegment y() {
            return y;
        }
        public double h() {
            return h;
        }
        public double arcLen() {
            return arcLen;
        }
    }
    private static class VelocityProfile {
        final private int SAMPLES;
        final private CompleteSpline spline;
        private double vMax, aMax, aLatMax;
        private double vi, vf;

        private double[] s;

        public VelocityProfile(CompleteSpline reference, int samples, double vm, double am, double alm) {
            this.spline = reference;
            this.SAMPLES = samples;
            this.vMax = vm;
            this.aMax = am;
            this.aLatMax = alm;
        }

        public VelocityProfile(CompleteSpline reference, double vm, double am, double alm) {
            this(reference, (int) (2 * reference.len), vm, am, alm);
        }

        public void setPathConstraints(double vi, double vf) {
            this.vi = vi;
            this.vf = vf;
        }

        public double[] build () {
            s = new double[SAMPLES];

            for (int i = 0; i < SAMPLES; i++) {
                s[i] = ((double) i / (SAMPLES - 1)) * spline.len;;
            }

            double[] vCeil = velocityCeiling(s);
            double[] vF = forwardPass(s, vCeil);
            double[] vB = backwardPass(s, vCeil);

            double[] v = new double[SAMPLES];

            for (int i = 0; i < SAMPLES; i++) {
                v[i] = Math.min(vF[i], vB[i]);
            }

            return v;
        }

        private double[] velocityCeiling (double[] s) {
            double[] vc = new double[s.length];

            double e = 1e-9;
            for (int i = 0; i < s.length; i++) {
                double vCentripetal = Math.sqrt(aLatMax / Math.abs(spline.curvature(s[i]) + e));
                vc[i] = Math.min(vMax, vCentripetal); //TODO: wheel limiting with IK
            }

            return vc;
        }
        private double[] forwardPass (double[] s, double[] vCeil) {
            double[] v = new double[s.length];

            v[0] = vi;
            for (int i = 1; i < s.length; i++) {
                double fwd = Math.sqrt((v[i - 1] * v[i - 1]) + (2 * aMax * (s[i] - s[i - 1]))); //v^2 = v0^2 + 2adx
                v[i] = Math.min(vCeil[i], fwd);
            }

            return v;
        }
        private double[] backwardPass (double[] s, double[] vCeil) {
            double[] v = new double[s.length];

            v[s.length - 1] = vf;
            for (int i = s.length - 2; i >= 0; i--) {
                double bwd = Math.sqrt((v[i + 1] * v[i + 1]) + (2 * aMax * (s[i + 1] - s[i])));
                v[i] = Math.min(vCeil[i], bwd);
            }

            return v;
        }

        public double[] getS () {
            return s;
        }
    }

    private final SplineSegment[] segments;
    private final double[] segmentStarts;
    private final double len;
    private HeadingProfile heading;
    private final int SAMPLE_RATE = 20;
    private PathState[] samples;
    private double maxVel, maxAccel, maxLatAccel;
    private double vi, vf;

    private CompleteSpline (SplineSegment[] segments) {
        this.segments = segments;
        segmentStarts = new double[segments.length];

        segmentStarts[0] = 0;
        for (int i = 1; i < segmentStarts.length; i++) {
            segmentStarts[i] = segmentStarts[i - 1] + segments[i - 1].arcLen();
        }

        len = segmentStarts[segmentStarts.length - 1] + segments[segments.length - 1].arcLen();
    }

    private void setHeading(HeadingProfile heading) {
        this.heading = heading;
    }
    private void setVelocityConstraints(double maxVel, double maxAccel, double maxLatAccel) {
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.maxLatAccel = maxLatAccel;
    }
    private void setPathConstraints(double vi, double vf) {
        this.vi = vi;
        this.vf = vf;
    }
    private void computeSamples() {
        int nSeg = segments.length;
        int nSamp = SAMPLE_RATE * nSeg + 1;

        samples = new PathState[nSamp];

        VelocityProfile velocityProfile = new VelocityProfile(this, nSamp, maxVel, maxAccel, maxLatAccel);
        velocityProfile.setPathConstraints(vi, vf);
        double[] vSamples = velocityProfile.build();
        double[] sSamples = velocityProfile.getS();

        double t = 0;

        for (int i = 0; i < nSamp; i++) {
            double s = sSamples[i];

            int index = findSeg(s);
            double[] values = segments[index].sampleAll(s - segmentStarts[index]);

            samples[i] = new PathState(
                    new Pose2d(values[0], values[1]),
                    vSamples[i],
                    values[2],
                    values[3],
                    s,
                    t
            );

            if (i < nSamp - 1) {
                t += (2 * (sSamples[i + 1] - s)) / (vSamples[i] + vSamples[i + 1]);
            }
        }
    }

    public Pose2d sample (double arcLen) {
        if (arcLen > len) return null;
        int index = findSeg(arcLen);

        return segments[index].interpolate(arcLen - segmentStarts[index]);
    }
    public Pose2d samplePercent (double p) {
        if (p > 1) return null;

        return sample(p * len);
    }

    public double curvature (double s) {
        if (s > len) return 0.0;
        int index = findSeg(s);

        return segments[index].curvature(s - segmentStarts[index]);
    }
    public double pathDir (double s) {
        if (s > len) return 0.0;
        int index = findSeg(s);

        return segments[index].heading(s - segmentStarts[index]);
    }

    private int findSeg (double s) {
        int low = 0, high = segmentStarts.length - 1;
        while (low < high) {
            int mid = low + ((int) (high - low) / 2);
            if (segmentStarts[mid] > s) {
                high = mid - 1;
            } else {
                low = mid + 1;
            }
        }

        return high;
    }

    public double getLen() {
        return len;
    }
    public PathState[] getArray() {
        return samples;
    }
}
