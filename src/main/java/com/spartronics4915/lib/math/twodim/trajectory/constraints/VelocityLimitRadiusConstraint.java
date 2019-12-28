package com.spartronics4915.lib.math.twodim.trajectory.constraints;

import com.spartronics4915.lib.math.twodim.geometry.Pose2dWithCurvature;
import com.spartronics4915.lib.math.twodim.geometry.Translation2d;

public class VelocityLimitRadiusConstraint implements TimingConstraint<Pose2dWithCurvature> {

    /** Meters */
    private final Translation2d mCenter;
    /** Meters */
    private final double mRadius;
    /** Meters/sec */
    private final double mVelocityLimit;

    public VelocityLimitRadiusConstraint(Translation2d centerMeters, double radiusMeters, double velocityLimitMetersPerSec) {
        mCenter = centerMeters;
        mRadius = radiusMeters;
        mVelocityLimit = velocityLimitMetersPerSec;
    }

    @Override
    public double getMaxVelocity(Pose2dWithCurvature state) {
        return state.getTranslation().getDistance(mCenter) <= mRadius ? mVelocityLimit : Double.POSITIVE_INFINITY;
    }

    @Override
    public MinMaxAcceleration getMinMaxAcceleration(Pose2dWithCurvature state, double velocity) {
        return TimingConstraint.MinMaxAcceleration.kNoLimits;
    }
}