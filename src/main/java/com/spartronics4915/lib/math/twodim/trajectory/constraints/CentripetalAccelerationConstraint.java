package com.spartronics4915.lib.math.twodim.trajectory.constraints;

import com.spartronics4915.lib.math.twodim.geometry.Pose2dWithCurvature;

public class CentripetalAccelerationConstraint implements TimingConstraint<Pose2dWithCurvature> {

    /** Meters/sec^2 */
    private final double mMaxCentripetalAccel;

    public CentripetalAccelerationConstraint(final double maxCentripetalAccelMetersPerSecSq) {
        mMaxCentripetalAccel = maxCentripetalAccelMetersPerSecSq;
    }

    @Override
    public double getMaxVelocity(final Pose2dWithCurvature state) {
        return Math.sqrt(Math.abs(mMaxCentripetalAccel / state.getCurvature()));
    }

    @Override
    public MinMaxAcceleration getMinMaxAcceleration(final Pose2dWithCurvature state, final double velocity) {
        return MinMaxAcceleration.kNoLimits;
    }
}
