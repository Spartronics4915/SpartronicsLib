package com.spartronics4915.lib.sensors;

import com.spartronics4915.lib.math.geometry.Pose2d;

import org.junit.jupiter.api.Test;

public class TestT265Camera
{

    @Test
    public void TestNewCamera()
    {
        new T265Camera((Pose2d p, T265Camera.PoseConfidence c) -> System.out.println(p + " " + c), "foo", new Pose2d(), 0f);
    }

}
