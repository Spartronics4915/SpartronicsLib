package com.spartronics4915.lib.math.twodim.trajectory.types;

import com.spartronics4915.lib.math.Util;
import com.spartronics4915.lib.math.twodim.trajectory.types.Trajectory.TrajectorySamplePoint;

/**
 * This class makes it easy to walk a trajectory by some unit. This is abstract
 * so that subclasses can be made that specify which unit they use to index the
 * trajectory (note that this hierarchy is merely for readability, and not a
 * property of the subclasses enforced by the type system).
 * 
 * @param <S> State that the trajectory holds.
 */
public abstract class TrajectoryIterator<S extends State<S>> {

    private final Trajectory<S> mTrajectory;
    private double mProgress;
    private TrajectorySamplePoint<S> mCurrentSample;

    public TrajectoryIterator(final Trajectory<S> trajectory) {
        mTrajectory = trajectory;

        mCurrentSample = mTrajectory.sample(mTrajectory.getFirstInterpolant());
        mProgress = mTrajectory.getFirstInterpolant();
    }

    public boolean isDone() {
        return getRemainingProgress() == 0.0;
    }

    public double getProgress() {
        return mProgress;
    }

    public double getRemainingProgress() {
        return Math.max(0.0, mTrajectory.getLastInterpolant() - mProgress);
    }

    public TrajectorySamplePoint<S> getCurrentSample() {
        return mCurrentSample;
    }

    public Trajectory<S> getTrajectory() {
        return mTrajectory;
    }

    public TrajectorySamplePoint<S> advance(double additionalProgress) {
        mProgress = Util.limit(mProgress + additionalProgress, mTrajectory.getFirstInterpolant(),
                mTrajectory.getLastInterpolant());
        mCurrentSample = mTrajectory.sample(mProgress);
        return mCurrentSample;
    }

    public TrajectorySamplePoint<S> preview(double additionalProgress) {
        double progress = Util.limit(mProgress + additionalProgress, mTrajectory.getFirstInterpolant(),
                mTrajectory.getLastInterpolant());
        return mTrajectory.sample(progress);
    }
}