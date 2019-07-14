package com.spartronics4915.lib.hardware.sensors;

import java.nio.file.Paths;
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

        // Cleanup is quite tricky for us, because the native code has no idea when Java
        // will be done. This is why we can't use smart pointers in the native code.
        // Even worse, trying to cleanup with atexit in the native code is too late and
        // results in unfinished callbacks blocking. As a result a shutdown hook is our
        // best option.
        Runtime.getRuntime().addShutdownHook(new Thread(() -> T265Camera.cleanup()));
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

        /**
         * The robot's pose in meters.
         */
        public final Pose2d pose;
        /**
         * The robot's velocity in meters/sec and radians/sec.
         */
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
     * @param robotOffset        Offset of the center of the robot from the center
     *                           of the camera.
     * @param odometryCovariance Covariance of the odometry input when doing
     *                           sensor fusion (you probably want to tune this).
     */
    public T265Camera(Pose2d robotOffset, double odometryCovariance)
    {
        this(robotOffset, odometryCovariance, "");
    }

    /**
     * This method constructs a T265 camera and sets it up with the right info.
     * {@link T265Camera#start() start} will not be called, you must call it
     * yourself. Unless you want memory leaks you must call
     * {@link T265Camera#free() free} when you're done with this class. The
     * garbage collector will not help you.
     * 
     * @param robotOffset        Offset of the center of the robot from the center
     *                           of the camera. Units are meters.
     * @param odometryCovariance Covariance of the odometry input when doing
     *                           sensor fusion (you probablywant to tune this)
     * @param relocMapPath       path (including filename) to a relocalization map
     *                           to load.
     */
    public T265Camera(Pose2d robotOffset, double odometryCovariance, String relocMapPath)
    {
        mNativeCameraObjectPointer = newCamera(relocMapPath);
        setOdometryInfo((float) robotOffset.getTranslation().x(), (float) robotOffset.getTranslation().y(),
                (float) robotOffset.getRotation().getDegrees(), odometryCovariance);
    }

    /**
     * This allows the user-provided pose recieve callback to recieve data.
     * This will also reset the camera's pose to (0, 0) at 0 degrees.
     * <p>
     * This will not restart the camera following exportRelocalizationMap. You will
     * have to call {@link T265Camera#free()} and make a new {@link T265Camera}.
     * This is related to what appears to be a bug in librealsense.
     * 
     * @param poseConsumer A method to be called every time we recieve a pose from
     *                     <i>from a different thread</i>! You must synchronize
     *                     memory access accross threads!
     *                     <p>
     *                     Recieved poses are in meters.
     */
    public synchronized void start(Consumer<CameraUpdate> poseConsumer)
    {
        if (mIsStarted)
            throw new RuntimeException("T265 camera is already started");
        setPose(Pose2d.identity());
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
     * @param sensorId You can have multiple separate wheel sensors with different
     *                 numbers; set this to 0 if you don't care.
     * @param velocity The robot's translational velocity in meters/sec.
     */
    public void sendOdometry(int sensorId, Twist2d velocity)
    {
        Pose2d transVel = Pose2d.exp(velocity);
        sendOdometryRaw(sensorId, (float) transVel.getTranslation().x(), (float) transVel.getTranslation().y());
    }

    /**
     * This zeroes the camera pose to the provided new pose.
     * 
     * @param newPose The pose the camera should be zeroed to.
     */
    public synchronized void setPose(Pose2d newPose)
    {
        mZeroingOffset = newPose.transformBy(new Pose2d(mLastRecievedPose.getTranslation().inverse(), mLastRecievedPose.getRotation().inverse()));
    }

    /**
     * This will free the underlying native objects. You probably don't want to use
     * this; on program shutdown the native code will gracefully stop and delete any
     * remaining objects.
     */
    public native void free();

    private native void setOdometryInfo(float robotOffsetX, float robotOffsetY, float robotOffsetRads, double measurementCovariance);

    private native void sendOdometryRaw(int sensorId, float xVel, float yVel);

    private native long newCamera(String mapPath);

    private static native void cleanup();

    private synchronized void consumePoseUpdate(float x, float y, float radians, float dx, float dtheta, int confOrdinal)
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

        final Pose2d currentPose = new Pose2d(x, y, Rotation2d.fromRadians(radians));
        final Pose2d transformedPose =
                new Pose2d(currentPose.getTranslation().translateBy(mZeroingOffset.getTranslation()).rotateBy(mZeroingOffset.getRotation()),
                        currentPose.getRotation().rotateBy(mZeroingOffset.getRotation()));
        mPoseConsumer.accept(new CameraUpdate(transformedPose, new Twist2d(dx, 0.0, dtheta), confidence));

        mLastRecievedPose = currentPose;
    }

    /**
     * Thrown if something goes wrong in the native code
     */
    public static class CameraJNIException extends RuntimeException
    {

        // This must be static _and_ have this constructor if you want it to be
        // thrown from native code
        public CameraJNIException(String message)
        {
            super(message);
        }
    }
}
