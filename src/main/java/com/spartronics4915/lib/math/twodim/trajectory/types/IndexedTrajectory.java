package com.spartronics4915.lib.math.twodim.trajectory.types;

import java.util.List;
import java.util.stream.Stream;

import com.spartronics4915.lib.math.Util;

public class IndexedTrajectory<S extends State<S>> extends Trajectory<S> {

    public IndexedTrajectory(List<S> points) {
        super(points);
    }

	@Override
    public TrajectorySamplePoint<S> sample(double interpolant) {
        if (mPoints.isEmpty())
            throw new IndexOutOfBoundsException("Trajectory is empty");
        
        if (interpolant <= 0) {
            return new TrajectorySamplePoint<S>(getPoint(0));
        } else if (interpolant >= mPoints.size() - 1) {
            return new TrajectorySamplePoint<S>(getPoint(mPoints.size() - 1));
        } else {
            int index = (int) Math.floor(interpolant);
            double percent = interpolant - index;
            
            // 254 (and FalconLib) use Double.MIN_VALUE as an epsilon, so we're
            // going to as well to ensure consistency
            if (Util.epsilonEquals(percent, 0, Double.MIN_VALUE)) {
                return new TrajectorySamplePoint<S>(getPoint(index));
            } else if (Util.epsilonEquals(percent, 1, Double.MIN_VALUE)) {
                return new TrajectorySamplePoint<S>(getPoint(index + 1));
            } else {
                return new TrajectorySamplePoint<S>(
                    mPoints.get(index).interpolate(mPoints.get(index), percent),
                    index,
                    index + 1
                );
            }
        }
    }

    @Override
    public double getFirstInterpolant() {
        return 0;
    }

    @Override
    public double getLastInterpolant() {
        return Math.max(0.0, mPoints.size() - 1);
    }

    public Stream<S> stream() {
        return mPoints.stream();
    }

    public static class IndexedIterator<S extends State<S>> extends TrajectoryIterator<S> {
        public IndexedIterator(IndexedTrajectory<S> trajectory) {
            super(trajectory);
        }
    }
}