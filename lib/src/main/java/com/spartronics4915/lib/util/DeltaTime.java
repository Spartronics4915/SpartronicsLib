package com.spartronics4915.lib.util;

public class DeltaTime
{
    /** Seconds */
    private double mStartTime = -1;
    /** Seconds */
    double mCurrentTime = -1;
    /** Seconds */
    double mDeltaTime = 0.0;

    public double getDeltaTimeSeconds()
    {
        return mDeltaTime;
    }

    public double getCurrentTimeSeconds()
    {
        return mCurrentTime;
    }

    public double getStartTime()
    {
        if (mStartTime < 0)
            throw new RuntimeException("Can't get start time if DeltaTime has never been updated");

        return mStartTime;
    }

    public double updateTime(double newTimeSeconds)
    {
        if (mCurrentTime < 0)
            mDeltaTime = 0;
        else
            mDeltaTime = newTimeSeconds - mCurrentTime;

        mCurrentTime = newTimeSeconds;
        return mDeltaTime;
    }

    public void reset()
    {
        mCurrentTime = -1;
    }
}
