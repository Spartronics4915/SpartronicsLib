package com.spartronics4915.lib.math.twodim.trajectory.constraints;

import com.spartronics4915.lib.math.twodim.geometry.Pose2dWithCurvature;

public class AngularAccelerationConstraint implements TimingConstraint<Pose2dWithCurvature> {

    /** Rads/sec^2 */
    private final double mMaxAngularAcceleration;

    public AngularAccelerationConstraint(double maxAngularAccelRadsPerSecSq) {
        if (maxAngularAccelRadsPerSecSq < 0)
            throw new RuntimeException("Cannot have negative angular acceleration");

        mMaxAngularAcceleration = maxAngularAccelRadsPerSecSq;
    }

    @Override
    public double getMaxVelocity(Pose2dWithCurvature state) {
        /*
         * This bit ensures that we don't violate our constraint indirectly. I.e. we
         * don't want v^2 * dk/ds alone to go over the max angular acceleration.
         * 
         * v^2 * dk/ds = maxAngularAcceleration when linear acceleration = 0.
         * 
         * v = sqrt(maxAngularAcceleration / dk/ds)
         */

        return Math.sqrt(mMaxAngularAcceleration / Math.abs(state.getDCurvatureDs()));
    }

    @Override
    public MinMaxAcceleration getMinMaxAcceleration(Pose2dWithCurvature state, double velocity) {
        // @formatter:off
        /*
         * We want to limit the acceleration such that we never go above the specified angular acceleration.
         *
         * Angular acceleration = dw/dt     WHERE   w = omega = angular velocity
         * w = v * k                        WHERE   v = linear velocity, k = curvature
         *                                  WHERE   s = distance traveled
         *
         * dw/dt = d/dt (v * k)
         *
         * By chain rule,
         * dw/dt = dv/dt * k + v * dk/dt   [1]
         *
         * We don't have dk/dt, but we do have dk/ds and ds/dt
         * dk/ds * ds/dt = dk/dt     [2]
         *
         * Substituting [2] in [1], we get
         * dw/dt = acceleration * curvature + velocity * velocity * d_curvature
         * WHERE acceleration = dv/dt, velocity = ds/dt or v, d_curvature = dk/dt and curvature = k
         *
         * We now want to find the linear acceleration such that the angular acceleration (dw/dt) never goes above
         * the specified amount.
         *
         * We can rearrange the last equality we derived to get the below:
         * linear acceleration * curvature = dw/dt - (velocity * velocity * d_curvature)
         * linear acceleration = (dw/dt - (velocity * velocity * d_curvature)) / curvature
         *
         * (Above is adapted from a comment originally in FalconLib by Prateek Machiraju)
         */
        // @formatter:on

        double maxAbsoluteAccel = Math.abs(
            (mMaxAngularAcceleration - (velocity * velocity * state.getDCurvatureDs())) / state.getCurvature()
        );

        return new TimingConstraint.MinMaxAcceleration(-maxAbsoluteAccel, maxAbsoluteAccel);
    }

}