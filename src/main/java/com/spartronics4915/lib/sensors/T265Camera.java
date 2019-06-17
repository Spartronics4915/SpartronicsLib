package com.spartronics4915.lib.sensors;

import java.nio.file.Paths;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

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

    public static enum PoseConfidence
    {
        Failed,
        Low,
        Medium,
        High,
    }

    public static class CameraUpdate
    {
        public final Pose2d pose;
        public final Twist2d velocity;
        public final PoseConfidence confidence;

        public CameraUpdate(Pose2d pose, Twist2d velocity, PoseConfidence confidence)
        {
            this.pose = pose;
            this.velocity = velocity;
            this.confidence = confidence;
        }
    }

    private long mNativeCameraObjectPointer = 0;
    private boolean mIsStarted = false;
    private Pose2d mZeroingOffset = Pose2d.identity();
    private Pose2d mLastRecievedPose = Pose2d.identity();
    private Consumer<CameraUpdate> mPoseConsumer = null;

    /**
     * This method constructs a T265 camera and sets it up with the right info.
     * {@link T265Camera#start() start} will not be called, you must call it
     * yourself. Unless you want memory leaks you must call
     * {@link T265Camera#free() free} when you're done with this class. The
     * garbage collector will not help you.
     * 
     * @param cameraOffset       Offset of camera from center of robot
     * @param odometryCovariance Covariance of the odometry input when doing
     *                           sensor fusion (you probably want to tune this)
     */
    public T265Camera(Pose2d cameraOffset, float odometryCovariance)
    {
        this(cameraOffset, odometryCovariance, "");
    }

    /**
     * This method constructs a T265 camera and sets it up with the right info.
     * {@link T265Camera#start() start} will not be called, you must call it
     * yourself. Unless you want memory leaks you must call
     * {@link T265Camera#free() free} when you're done with this class. The
     * garbage collector will not help you.
     * 
     * @param cameraOffset       Offset of camera from center of robot
     * @param odometryCovariance Covariance of the odometry input when doing
     *                           sensor fusion (you probablywant to tune this)
     * @param relocMapPath       path (including filename) to a relocalization map
     *                           to load
     */
    public T265Camera(Pose2d cameraOffset, float odometryCovariance, String relocMapPath)
    {
        mNativeCameraObjectPointer = newCamera(relocMapPath);
        setOdometryInfo((float) cameraOffset.getTranslation().x(), (float) cameraOffset.getTranslation().y(),
                (float) cameraOffset.getRotation().getDegrees(), odometryCovariance);
    }

    /**
     * This allows the user-provided pose recieve callback to recieve data.
     * <p>
     * This will not restart the camera following exportRelocalizationMap. You will
     * have to call {@link T265Camera#free()} and make a new {@link T265Camera}.
     * This is related to what appears to be a bug in librealsense.
     * 
     * @param poseConsumer Called every time we recieve a pose from the
     *                     camera <i>from a different thread</i>! You must
     *                     synchronize memory access accross threads!
     */
    public synchronized void start(Consumer<CameraUpdate> poseConsumer)
    {
        mPoseConsumer = poseConsumer;
        mIsStarted = true;
    }

    /**
     * This allows the callback to recieve data, but it does not internally stop the
     * camera.
     */
    public synchronized void stop()
    {
        mIsStarted = false;
    }

    /**
     * Exports a binary relocalization map file to the given path.
     * This will stop the camera. Because of a librealsense bug the camera isn't be
     * restarted after you call this method. TODO: Fix that.
     * 
     * @param path Path (with filename) to export to
     */
    public native void exportRelocalizationMap(String path);

    /**
     * Sends robot velocity as computed from wheel encoders.
     * 
     * @param sensorId    TODO: What is this
     * @param frameNumber TODO: How to get this
     * @param velocity    The robot's translational velocity
     */
    public void sendOdometry(int sensorId, int frameNumber, Twist2d velocity)
    {
        Pose2d transVel = Pose2d.exp(velocity);
        sendOdometryRaw(sensorId, frameNumber, (float) transVel.getTranslation().x(), (float) transVel.getTranslation().y());
    }

    /**
     * This zeroes the camera pose to the provided new pose.
     * 
     * @param newPose The new pose the camera should be at.
     */
    public synchronized void setPose(Pose2d newPose)
    {
        mZeroingOffset = newPose.transformBy(mLastRecievedPose.inverse());
    }

    /**
     * This must be called when you're done with this class or you will get memory
     * leaks.
     */
    public native void free();

    private native void setOdometryInfo(float camOffsetX, float camOffsetY, float camOffsetRads, float measurementCovariance);

    private native void sendOdometryRaw(int sensorId, int frameNumber, float xVel, float yVel);

    private native long newCamera(String mapPath);

    private synchronized void consumePoseUpdate(float x, float y, float radians, float xVel, int confOrdinal)
    {
        if (!mIsStarted)
            return;

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
        mPoseConsumer.accept(new CameraUpdate(new Pose2d(x, y, Rotation2d.fromRadians(radians)).transformBy(mZeroingOffset), new Twist2d(xVel, 0.0, 0.0), confidence));
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
}
