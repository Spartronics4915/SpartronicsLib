package com.spartronics4915.lib.sensors;

import java.nio.file.Paths;
import java.util.function.BiConsumer;

import com.spartronics4915.lib.math.geometry.Pose2d;
import com.spartronics4915.lib.math.geometry.Rotation2d;
import com.spartronics4915.lib.math.geometry.Twist2d;

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

    /**
     * Thrown if something goes wrong in the native code
     */
    public static class CameraJNIException extends Exception
    {
        // This must be static _and_ have this constructor if you want it to be
        // thrown from native code
        public CameraJNIException(String message)
        {
            super(message);
        }
    }

    public static enum PoseConfidence
    {
        Failed,
        Low,
        Medium,
        High,
    }

    private long mNativeCameraObjectPointer = 0;
    private final BiConsumer<Pose2d, PoseConfidence> mPoseConsumer;

    /**
     * This method constructs a T265 camera and sets it up with the right info.
     * <code>startCamera</code> must be called before you will get anything; it is
     * not called in the constructor. Unless you want memory leaks you must call
     * <code>free</code> when you're done with this class. The garbage collector
     * will not help you.
     * 
     * @param poseConsumer          called every time we recieve a pose from the
     *                              camera
     * @param relocalizationMapPath path (in the filesystem) to a relocalization map
     *                              (you can get one via
     *                              <code>exportRelocalizationMap</code>)
     * @param cameraOffset          offset of camera from center of robot
     * @param odometryCovariance    covariance of the odometry input when doing
     *                              sensor fusion (you probably tune this)
     */
    public T265Camera(BiConsumer<Pose2d, PoseConfidence> poseConsumer,
            String relocalizationMapPath, Pose2d cameraOffset, float odometryCovariance)
    {
        mPoseConsumer = poseConsumer;
        mNativeCameraObjectPointer = newCamera();
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
    private native long newCamera();
    // TODO: Add exportRelocalizationMap
    private native void loadRelocalizationMap(String path);
    private native void setOdometryInfo(float camOffsetX, float camOffsetY, float camOffsetRads, float measurementCovariance);
    private native void sendOdometryRaw(int sensorId, int frameNumber, float xVel, float yVel);

    private void consumePoseUpdate(float x, float y, float radians, int confOrdinal)
    {
        // See https://github.com/IntelRealSense/librealsense/blob/7f2ba0de8769620fd672f7b44101f0758e7e2fb3/include/librealsense2/h/rs_types.h#L115 for ordinals
        PoseConfidence confidence;
        switch (confOrdinal)
        {
            case 0x0:
                confidence = PoseConfidence.Failed;
                break;
            case 0x1:
                confidence = PoseConfidence.Low;
                break;
            case 0x2:
                confidence = PoseConfidence.Medium;
                break;
            case 0x3:
                confidence = PoseConfidence.High;
                break;
            default:
                throw new RuntimeException("Unknown confidence ordinal \"" + confOrdinal + "\" passed from native code");
        }
        mPoseConsumer.accept(new Pose2d(x, y, Rotation2d.fromRadians(radians)), confidence);
    }
}
