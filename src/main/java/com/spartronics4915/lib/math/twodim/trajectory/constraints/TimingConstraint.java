package com.spartronics4915.lib.math.twodim.trajectory.constraints;

public interface TimingConstraint<S> {
    double getMaxVelocity(S state);

    MinMaxAcceleration getMinMaxAcceleration(S state, double velocity);

    public class MinMaxAcceleration {

        public static final MinMaxAcceleration kNoLimits =
            new MinMaxAcceleration(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

        /** Meters/sec^2 */
        public final double minAcceleration, maxAcceleration;

        /** Checks if minAcceleration <= maxAcceleration */
        public final boolean valid;

        public MinMaxAcceleration(double minAccelerationMetersPerSecSq, double maxAccelerationMetersPerSecSq) {
            minAcceleration = minAccelerationMetersPerSecSq;
            maxAcceleration = maxAccelerationMetersPerSecSq;
            valid = minAcceleration <= maxAcceleration;
        }
    }
}