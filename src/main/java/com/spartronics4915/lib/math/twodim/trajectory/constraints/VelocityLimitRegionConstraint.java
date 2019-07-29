package com.spartronics4915.lib.math.twodim.trajectory.constraints;

import com.spartronics4915.lib.math.twodim.geometry.Pose2dWithCurvature;
import com.spartronics4915.lib.math.twodim.geometry.Rectangle2d;

public class VelocityLimitRegionConstraint implements TimingConstraint<Pose2dWithCurvature> {

    /** Meters */
    private Rectangle2d mRegion;
    /** Meters/sec */
    private double mVelocityLimit;

    public VelocityLimitRegionConstraint(Rectangle2d regionMeters, double velocityLimitMetersPerSec) {
        mRegion = regionMeters;
        mVelocityLimit = velocityLimitMetersPerSec;
    }

    @Override
    public double getMaxVelocity(Pose2dWithCurvature state) {
        return mRegion.contains(state.getTranslation()) ? mVelocityLimit : Double.POSITIVE_INFINITY;
    }

    @Override
    public MinMaxAcceleration getMinMaxAcceleration(Pose2dWithCurvature state, double velocity) {
        return TimingConstraint.MinMaxAcceleration.kNoLimits;
    }

}