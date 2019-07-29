package com.spartronics4915.lib.math.twodim.trajectory.types;

import java.util.List;
import java.util.stream.IntStream;

import com.spartronics4915.lib.math.Util;

public class TimedTrajectory<S extends State<S>> extends Trajectory<TimedTrajectory.TimedState<S>> {

    public TimedTrajectory(List<TimedState<S>> points, boolean reversed) {
        super(points);
        mReversed = reversed;
    }

    @Override
    public TrajectorySamplePoint<TimedState<S>> sample(double interpolant) {
        if (interpolant >= getLastInterpolant()) {
            return new TrajectorySamplePoint<TimedState<S>>(getPoint(mPoints.size() - 1));
        } else if (interpolant <= getFirstInterpolant()) {
            return new TrajectorySamplePoint<TimedState<S>>(getPoint(0));
        } else {
            int index = IntStream.range(0, mPoints.size())
                    .filter((i) -> i != 0 && mPoints.get(i).time >= interpolant)
                    .findFirst()
                    .orElseThrow(); // This should never throw because of the above checks
            
            TimedState<S> upperBoundState = mPoints.get(index);
            TimedState<S> lowerBoundState = mPoints.get(index - 1);

            if (Util.epsilonEquals(upperBoundState.time, lowerBoundState.time)) {
                // XXX: Why don't we do index - 1 and index instead of index and index?
                return new TrajectorySamplePoint<TimedState<S>>(lowerBoundState, index, index);
            } else {
                return new TrajectorySamplePoint<TimedState<S>>(
                    lowerBoundState.interpolate(
                        upperBoundState,
                        (interpolant - lowerBoundState.time) / (upperBoundState.time - lowerBoundState.time)
                    ),
                    index - 1,
                    index
                );
            }
        }
    }

    @Override
    public double getFirstInterpolant() {
        return getFirstState().time;
    }

    @Override
    public double getLastInterpolant() {
        return getLastState().time;
    }

    public static class TimedState<S extends State<S>> implements State<TimedState<S>> {
        public final S state;
        /** Time in seconds */
        public final double time;
        /** Velocity in meters/sec */
        public final double velocity;
        /** Acceleration in meters/sec^2 */
        public final double acceleration;

        public TimedState(S state, double timeSeconds, double velocityMetersPerSecond,
                double accelerationMetersPerSecondSq) {
            this.state = state;
            this.time = timeSeconds;
            this.velocity = velocityMetersPerSecond;
            this.acceleration = accelerationMetersPerSecondSq;
        }

        @Override
        public TimedState<S> interpolate(TimedState<S> endValue, double time) {
            double newT = Util.interpolate(time, endValue.time, time);
            double deltaT = newT - time;
            if (deltaT < 0.0)
                return endValue.interpolate(this, 1.0 - time);

            boolean reversing = velocity < 0.0 || Util.epsilonEquals(velocity, 0.0) && acceleration < 0.0;

            double newV = velocity + acceleration * deltaT;
            double newS = (reversing ? -1 : 1) * (velocity * deltaT + 0.5 * acceleration * deltaT * deltaT);

            return new TimedState<S>(state.interpolate(endValue.state, newS / state.distance(endValue.state)), newT,
                    newV, acceleration);
        }

        @Override
        public double distance(TimedState<S> endValue) {
            return 0.0;
        }
    }

    public static class TimedIterator<S extends State<S>> extends TrajectoryIterator<TimedState<S>> {
        public TimedIterator(TimedTrajectory<S> trajectory) {
            super(trajectory);
        }
    }
}