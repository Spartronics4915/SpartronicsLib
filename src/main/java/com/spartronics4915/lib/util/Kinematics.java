package com.spartronics4915.lib.util;

import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;
import com.spartronics4915.lib.math.twodim.geometry.Twist2d;

/**
 * Provides forward and inverse kinematics equations for the robot modeling the
 * wheelbase as a differential drive (with a corrective factor to account for
 * skidding).
 */
public class Kinematics
{
    // TODO: Deduplicate kinematics once we add in path following stuff

    private final double mTrackWidthInches, mScrubFactor;

    public Kinematics(double trackWidthInches, double scrubFactor)
    {
        mTrackWidthInches = trackWidthInches;
        mScrubFactor = scrubFactor;
    }

    /**
     * Forward kinematics using only encoders, rotation is implicit (less accurate
     * than below, but useful for predicting motion)
     */
    public Twist2d forwardKinematics(double leftWheelDelta, double rightWheelDelta)
    {
        double delta_rotation = (rightWheelDelta - leftWheelDelta)
                / (mTrackWidthInches * mScrubFactor);
        return forwardKinematics(leftWheelDelta, rightWheelDelta, delta_rotation);
    }

    public Twist2d forwardKinematics(double leftWheelDelta, double rightWheelDelta,
            double delta_rotation_rads)
    {
        final double dx = (leftWheelDelta + rightWheelDelta) / 2.0;
        return new Twist2d(dx, 0.0, delta_rotation_rads);
    }

    public Twist2d forwardKinematics(Rotation2d prevHeading, double leftWheelDelta, double rightWheelDelta,
            Rotation2d currentHeading)
    {
        final double dx = (leftWheelDelta + rightWheelDelta) / 2.0;
        final double dy = 0.0;
        return new Twist2d(dx, dy, prevHeading.inverse().rotateBy(currentHeading).getRadians());
    }

    /**
     * For convenience, integrate forward kinematics with a Twist2d and previous
     * rotation.
     */
    public Pose2d integrateForwardKinematics(Pose2d currentPose, Twist2d forward_kinematics)
    {
        return currentPose.transformBy(Pose2d.exp(forward_kinematics));
    }
}
