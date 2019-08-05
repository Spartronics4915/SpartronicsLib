package com.spartronics4915.lib.math.twodim.control;

import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.math.twodim.geometry.Pose2dWithCurvature;
import com.spartronics4915.lib.math.twodim.trajectory.types.TimedTrajectory.TimedIterator;

/**
 * This tracker follows without any external disturbance correction based on
 * odometry. Hence it feeds the path's target velocities and accelerations
 * forward to the drivetrain, without modification.
 */
public class FeedForwardTracker extends TrajectoryTracker {

    @Override
	protected TrajectoryTrackerVelocityOutput calculateState(
        TimedIterator<Pose2dWithCurvature> iterator,
        Pose2d currentRobotPoseMeters
    ) {
        var referenceState = iterator.getCurrentSample().state;

        double vLinear, vAngular;
        vLinear = referenceState.velocity;
        vAngular = vLinear * referenceState.state.getCurvature();

		return new TrajectoryTrackerVelocityOutput(
            vLinear,
            vAngular
        );
	}

}