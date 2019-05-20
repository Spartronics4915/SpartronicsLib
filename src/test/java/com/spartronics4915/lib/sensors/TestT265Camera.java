package com.spartronics4915.lib.sensors;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.concurrent.CountDownLatch;

import com.spartronics4915.lib.math.geometry.Pose2d;
import com.spartronics4915.lib.math.geometry.Twist2d;
import com.spartronics4915.lib.util.Logger;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

public class TestT265Camera
{

    private boolean mDataRecieved = false;

    @Tag("hardwareDependant")
    @Test
    public void TestNewCamera()
    {
        // This one is a little hard to unit test because we haven't simulated the hardware
        // We mostly just make sure that we can get through this sequence without throwing an exception
        // The rest is just a few sanity checks

        T265Camera cam = new T265Camera((Pose2d p, T265Camera.PoseConfidence c) -> {
            mDataRecieved = true;
            Logger.debug("Got pose " + p + " with confidence " + c);
        }, new Pose2d(), 0f);
        cam.startCamera();
        cam.sendOdometry(0, 0, new Twist2d(3, 0, 1));

        Logger.debug("Waiting 10 seconds to recieve data...");
        try
        {
            Thread.sleep(10000);
        }
        catch (InterruptedException e)
        {
            Logger.exception(e);
        }
        assertTrue(mDataRecieved, "No pose data was recieved after 10 seconds... Try moving the camera?");

        Path path = Paths.get(System.getProperty("java.io.tmpdir"), "map.bin").toAbsolutePath();
        cam.exportRelocalizationMap(path.toString());

        if (path.toFile().length() <= 0)
            fail("Relocalization map file length was 0");

        cam.stopCamera();
        cam.free();
    }

}
