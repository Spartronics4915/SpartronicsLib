package com.spartronics4915.lib.util;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;

/**
 * Provides forward and inverse kinematics equations for the robot modeling the
 * wheelbase as a differential drive (with a corrective factor to account for
 * skidding).
 */
public class Kinematics
{
    // TODO: Deduplicate kinematics once we add in path following stuff (i.e. remove this file)

    private final double mTrackWidthInches, mScrubFactor;

    public Kinematics(double trackWidthInches, double scrubFactor)
    {
        mTrackWidthInches = trackWidthInches;
        mScrubFactor = scrubFactor;
    }

    public Twist2d forwardKinematics(Rotation2d prevHeading, double leftWheelDelta, double rightWheelDelta,
                                     Rotation2d currentHeading)
    {
        final double dx = (leftWheelDelta + rightWheelDelta) / 2.0;
        final double dy = 0.0;
        return new Twist2d(dx, dy, prevHeading.unaryMinus().plus(currentHeading).getRadians());
    }

    /**
     * For convenience, integrate forward kinematics with a Twist2d and previous
     * rotation.
     */
    public Pose2d integrateForwardKinematics(Pose2d currentPose, Twist2d forwardKinematics)
    {
        return currentPose.exp(forwardKinematics);
    }
}
