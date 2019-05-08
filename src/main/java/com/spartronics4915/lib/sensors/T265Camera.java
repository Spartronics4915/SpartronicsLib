package com.spartronics4915.lib.sensors;

import com.spartronics4915.lib.math.geometry.Pose2d;
import com.spartronics4915.lib.math.geometry.Twist2d;

import java.nio.file.Paths;
import java.util.function.Supplier;

/**
 * Provides a convenient Java interface to the Intel RealSense
 * T265 V-SLAM camera. Only the subset of the librealsense that is useful
 * to robot tracking is exposed in this class.
 * 
 * We employ JNI to call librealsense. There _are_ Java bindings for
 * librealsense, but they are not complete and do not support our usecase.
 * 
 * This class works entirely in 2d, even though the tracking camera supports
 * giving us a third dimension.
 * 
 */
public class T265Camera
{

    static
    {
        // FIXME: Use System.loadLibrary
        System.load(Paths.get(System.getProperty("user.home"), "libspartronicsnative.so").toAbsolutePath().toString());
    }

    private long nativeCameraObjectPointer = 0;

    public T265Camera(Supplier<Pose2d> poseRecievedCallback,
            String relocalizationMapPath, Pose2d cameraOffset, float odometryCovariance)
    {
        nativeCameraObjectPointer = newCamera(poseRecievedCallback);
        loadRelocalizationMap(relocalizationMapPath);
        setOdometryInfo((float) cameraOffset.getTranslation().x(), (float) cameraOffset.getTranslation().y(),
                (float) cameraOffset.getRotation().getDegrees(), odometryCovariance);
    }

    public native void startCamera();
    public native void stopCamera();
    public void sendOdometry(int sensorId, int frameNumber, Twist2d velocity)
    {
        Pose2d transVel = Pose2d.exp(velocity);
        sendOdometryRaw(sensorId, frameNumber, (float) transVel.getTranslation().x(), (float) transVel.getTranslation().y());
    }

    /**
     * This must be called when you're done with this class or you will get memory
     * leaks.
     */
    public native void free();
    private native long newCamera(Supplier<Pose2d> poseSupplier);
    private native void loadRelocalizationMap(String path);
    private native void setOdometryInfo(float offsetX, float offsetY, float offsetAng, float measurementCovariance);
    private native void sendOdometryRaw(int sensorId, int frameNumber, float xVel, float yVel);

    /**
     * Thrown if something goes wrong in the native code
     */
    public static class CameraJNIException extends Exception
    {
        public CameraJNIException(String message)
        {
            super(message);
        }
    }
}
