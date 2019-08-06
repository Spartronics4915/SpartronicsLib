package com.spartronics4915.lib.math.twodim.trajectory.types;

import java.util.List;

import com.spartronics4915.lib.util.Interpolable;

public abstract class Trajectory<S extends Interpolable<S>> {
    protected List<S> mPoints;
    protected boolean mReversed = false;

    protected Trajectory(List<S> points) {
        mPoints = points;
    }

    public TrajectoryPoint<S> getPoint(int index) {
        return new TrajectoryPoint<S>(index, mPoints.get(index));
    }

    public int size() {
        return mPoints.size();
    }

    public S getFirstState() {
        return mPoints.get(0);
    }

    public S getLastState() {
        return mPoints.get(mPoints.size() - 1);
    }

    public boolean isReversed() {
        return mReversed;
    }

    public abstract TrajectorySamplePoint<S> sample(double interpolant);

    public abstract double getFirstInterpolant();
    public abstract double getLastInterpolant();

    public static class TrajectoryPoint<S> {
        public int index;
        public S state;

        public TrajectoryPoint(int index, S state) {
            this.index = index;
            this.state = state;
        }
    }

    public static class TrajectorySamplePoint<S> {
        public S state;
        public int indexFloor, indexCeil;

        public TrajectorySamplePoint(TrajectoryPoint<S> p) {
            this.state = p.state;
            this.indexFloor = p.index;
            this.indexCeil = p.index;
        }

        public TrajectorySamplePoint(S state, int indexFloor, int indexCeil) {
            this.state = state;
            this.indexFloor = indexFloor;
            this.indexCeil = indexCeil;
        }
    }
}