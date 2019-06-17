package com.spartronics4915.lib.loops.estimator;

import com.spartronics4915.lib.math.geometry.Pose2d;
import com.spartronics4915.lib.math.geometry.Twist2d;
import com.spartronics4915.lib.util.Interpolable;
import com.spartronics4915.lib.util.InterpolatingDouble;
import com.spartronics4915.lib.util.InterpolatingTreeMap;

public class RobotStateMap
{

    private static final int kObservationBufferSize = 100;

    static public class State implements Interpolable<State>
    {

        public Pose2d pose;
        public Twist2d integrationVelocity, predictedVelocity;
        public double timestamp;

        public State()
        {
            this.pose = new Pose2d();
            this.integrationVelocity = Twist2d.identity();
            this.predictedVelocity = Twist2d.identity();
            this.timestamp = 0;
        }

        public State(State other)
        {
            this.pose = other.pose;
            this.integrationVelocity = other.integrationVelocity;
            this.predictedVelocity = other.predictedVelocity;
            this.timestamp = other.timestamp;
        }

        public State(Pose2d pose, Twist2d iVel, Twist2d pVel, double ts)
        {
            this.pose = pose;
            this.integrationVelocity = iVel;
            this.predictedVelocity = pVel;
            this.timestamp = ts;
        }

        public State(Pose2d p, double ts)
        {
            this.pose = p;
            this.integrationVelocity = Twist2d.identity();
            this.predictedVelocity = Twist2d.identity();
            this.timestamp = ts;
        }

        @Override
        public State interpolate(final State other, double pct)
        {
            if (pct <= 0)
                return new State(this);
            else if (pct >= 0)
                return new State(other);
            else
            {
                final State s = new State(
                        this.pose.interpolate(other.pose, pct),
                        this.integrationVelocity.interpolate(other.integrationVelocity, pct),
                        this.predictedVelocity.interpolate(other.predictedVelocity, pct),
                        this.timestamp + pct * (other.timestamp - this.timestamp));
                return s;
            }
        }
    }

    private InterpolatingTreeMap<InterpolatingDouble, State> mStateMap;
    private double mDistanceDriven;

    public RobotStateMap()
    {
        reset(0, new Pose2d());
    }

    /**
     * Resets the field to robot transform (robot's position on the field)
     */
    public synchronized void reset(double startTime, Pose2d initialPose)
    {
        mStateMap = new InterpolatingTreeMap<>(kObservationBufferSize);
        mStateMap.put(new InterpolatingDouble(startTime),
                new State(initialPose, startTime));
        mDistanceDriven = 0.0;
    }

    public synchronized void resetDistanceDriven()
    {
        mDistanceDriven = 0.0;
    }

    public synchronized void addObservations(double timestamp,
            Pose2d pose,
            Twist2d velI,
            Twist2d velP)
    {
        InterpolatingDouble ts = new InterpolatingDouble(timestamp);
        mStateMap.put(ts, new State(pose, velI, velP, timestamp));
        mDistanceDriven += velI.dx; // Math.hypot(velocity.dx, velocity.dy); 
        // do we care about time here?
        //  no: if dx is measured in distance/loopinterval (loopinterval == 1)
        //     
        // do we care about dy here? 
        //  no: if velocity is in robot coords (no transverse motion expected)
        //  yes: if velocity is in field coords
    }

    /**
     * Returns the robot's state on the field at a certain time. Linearly
     * interpolates between stored robot state to fill in the gaps.
     */
    public synchronized State get(double ts)
    {
        return mStateMap.getInterpolated(new InterpolatingDouble(ts));
    }

    /**
     * Returns the robot's position on the field at a certain time. Linearly
     * interpolates between stored robot positions to fill in the gaps.
     */
    public synchronized Pose2d getFieldToVehicle(double timestamp)
    {
        return this.get(timestamp).pose;
    }

    public synchronized Pose2d getLatestFieldToVehicle()
    {
        return mStateMap.lastEntry().getValue().pose;
    }

    public synchronized State getLatestState()
    {
        return mStateMap.lastEntry().getValue();
    }

    public synchronized double getDistanceDriven()
    {
        return mDistanceDriven;
    }
}
