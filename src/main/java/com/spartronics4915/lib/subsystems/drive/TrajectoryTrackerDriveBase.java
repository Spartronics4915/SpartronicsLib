package com.spartronics4915.lib.subsystems.drive;

import com.spartronics4915.lib.hardware.motors.SpartronicsMotor;
import com.spartronics4915.lib.math.twodim.control.TrajectoryTracker;
import com.spartronics4915.lib.math.twodim.control.TrajectoryTracker.TrajectoryTrackerOutput;
import com.spartronics4915.lib.math.twodim.geometry.Pose2d;

/**
 * This is the minimal interface you need to implement if you want to use
 * TrajectoryTrackerCommand.
 */
public interface TrajectoryTrackerDriveBase {
    SpartronicsMotor getLeftMotor();

    SpartronicsMotor getRightMotor();

    void setOutput(TrajectoryTrackerOutput output);

    default void zeroOutputs() {
        getLeftMotor().setNeutral();
        getRightMotor().setNeutral();
    }
}