package com.spartronics4915.lib.math.twodim.control;

import com.spartronics4915.lib.math.twodim.geometry.Pose2dWithCurvature;
import com.spartronics4915.lib.math.twodim.trajectory.types.TimedTrajectory.TimedIterator;

public abstract class TrajectoryTracker {
        private TimedIterator<Pose2dWithCurvature> mTrajectoryIterator = null;
}