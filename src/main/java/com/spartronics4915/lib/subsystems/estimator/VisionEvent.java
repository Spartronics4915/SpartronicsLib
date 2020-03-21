package com.spartronics4915.lib.subsystems.estimator;

import edu.wpi.first.wpilibj.geometry.Pose2d;

public class VisionEvent implements Runnable
{
    public Pose2d mVisionEstimate = null;
    public double mTimestamp = 0;

    public void run()
    {
    }; // override me
}
