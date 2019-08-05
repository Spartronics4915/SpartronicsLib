package com.spartronics4915.lib.math.twodim.trajectory.types;

import java.util.ArrayList;
import java.util.List;
import java.util.ListIterator;
import java.util.stream.IntStream;

import com.spartronics4915.lib.math.Util;

public class DistancedTrajectory<S extends State<S>> extends Trajectory<S> {

    private List<Double> mDistances = new ArrayList<>();

    public DistancedTrajectory(List<S> points) {
        super(points);

        mDistances.add(0.0);

        ListIterator<S> iter = mPoints.listIterator();
        while (iter.hasNext()) {
            // TODO: Check this
            mDistances.add(mDistances.get(mDistances.size() - 1) + iter.next().distance(iter.next()));
            iter.previous(); // Send the iterator back by 1 (we're basically doing a peek)
        }
    }

    public DistancedTrajectory(IndexedTrajectory<S> otherTraj) {
        this(otherTraj.mPoints);
    }

    @Override
    public TrajectorySamplePoint<S> sample(double interpolant) {
        if (interpolant >= getLastInterpolant()) {
            return new TrajectorySamplePoint<S>(getPoint(mPoints.size() - 1));
        } else if (interpolant <= getFirstInterpolant()) {
            return new TrajectorySamplePoint<S>(getPoint(0));
        } else {
            int index = IntStream.range(0, mPoints.size()).filter((i) -> i != 0 && mDistances.get(i) >= interpolant)
                    .findFirst().orElseThrow(); // This should never throw because of the above checks

            S upperBoundState = mPoints.get(index);
            S lowerBoundState = mPoints.get(index - 1);

            if (Util.epsilonEquals(mDistances.get(index), mDistances.get(index - 1))) {
                // XXX: Why don't we do index - 1 and index instead of index and index?
                return new TrajectorySamplePoint<S>(lowerBoundState, index, index);
            } else {
                return new TrajectorySamplePoint<S>(
                    lowerBoundState.interpolate(
                        upperBoundState,
                        (interpolant - mDistances.get(index - 1)) / (mDistances.get(index) - mDistances.get(index - 1))
                    ),
                    index - 1, index
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
        return 0;
    }

    public static class DistancedIterator<S extends State<S>> extends TrajectoryIterator<S> {
        public DistancedIterator(DistancedTrajectory<S> trajectory) {
            super(trajectory);
        }
    }

}