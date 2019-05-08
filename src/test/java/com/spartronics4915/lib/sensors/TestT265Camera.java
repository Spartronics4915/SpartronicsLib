package com.spartronics4915.lib.sensors;

import com.spartronics4915.lib.math.geometry.Pose2d;

import org.junit.jupiter.api.Test;

public class TestT265Camera
{

    @Test
    public void TestNewCamera()
    {
        new T265Camera(() -> new Pose2d(), "foo", new Pose2d(), 0f);
    }

}
