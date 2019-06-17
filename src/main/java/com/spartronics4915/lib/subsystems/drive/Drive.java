package com.spartronics4915.lib.subsystems.drive;

import com.spartronics4915.lib.math.geometry.Rotation2d;
import com.spartronics4915.lib.subsystems.Subsystem;

public abstract class Drive extends Subsystem
{
    // TODO: Come up with a sane way to do things like velocity or open loop
    // TODO: Also come up with an abstract motor class so that we can do more stuff in here

    /**
     * This sets the heading of the IMU, but this should almost exclusively be
     * called by RobotStateEstimator, because that is the single source of truth for
     * robot heading.
     * 
     * @param heading Heading to set the IMU to.
     */
    public abstract void setIMUHeading(Rotation2d heading);

    /**
     * This gets the heading of the IMU, but this should almost exclusively be
     * called by RobotStateMap, which is the single source of truth for all matters
     * of robot pose (including heading).
     * 
     * @return Heading of the IMU.
     */
    public abstract Rotation2d getIMUHeading();

    public abstract double getLeftRotations();

    public abstract double getRightRotations();

    public abstract double getLeftDistanceInches();

    public abstract double getRightDistanceInches();

    public abstract double getLeftVelocityInchesPerSec();

    public abstract double getRightVelocityInchesPerSec();

    public abstract double getRobotLinearVelocity();

    public abstract double getRobotAngularVelocity();
}
