package com.spartronics4915.lib.hardware.sensors;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpiutil.RuntimeLoader;

/**
 * Provides a convenient Java interface to the Intel RealSense
 * T265 V-SLAM camera. Only the subset of the librealsense that is useful
 * to robot tracking is exposed in this class.
 * <p>
 * We employ JNI to call librealsense. There <i>are</i> Java bindings for
 * librealsense, but they are not complete and do not support our usecase.
 * <p>
 * This class works entirely in 2d, even though the tracking camera supports
 * giving us a third dimension (Z).
 * <p>
 * The coordinate system is as follows:
 * + X == Camera forwards
 * + Y == Camera Left (left is from the perspective of a viewer standing behind the camera)
 * <p>
 * All distance units are meters. All time units are seconds.
 */
public class T265Camera {
    private static boolean sLibraryLoaded = false;
    private static RuntimeLoader<T265Camera> sLoader = null;

    /**
     * A helper class to allow the user control over when the native code is extracted and loaded.
     */
    public static class Helper {
        private static AtomicBoolean sExtractOnStaticLoad = new AtomicBoolean(true);

        /**
         * Checks when the native code is extracted and loaded.
         *
         * @return True if the native code is extracted and loaded in a static block; false if on
         *         instantiation.
         */
        public static boolean getExtractOnStaticLoad() {
            return sExtractOnStaticLoad.get();
        }

        /**
         * Determines when the native code is extracted and loaded.
         *
         * @param load True if the native code is extracted and loaded in a static block; false if on
         *             instantiation.
         */
        public static void setExtractOnStaticLoad(boolean load) {
            sExtractOnStaticLoad.set(load);
        }
    }

    static {
        if (Helper.getExtractOnStaticLoad()) {
            try {
                sLoader = new RuntimeLoader<>("Spartronics", RuntimeLoader.getDefaultExtractionRoot(), T265Camera.class);
                sLoader.loadLibrary();
            } catch (IOException ex) {
                ex.printStackTrace();
                System.exit(1);
            }
            sLibraryLoaded = true;
        }
    }

    /**
     * Force load the library.
     *
     * @throws IOException thrown if the native library cannot be found
     */
    public static synchronized void forceLoad() throws IOException {
        if (sLibraryLoaded) {
            return;
        }
        sLoader = new RuntimeLoader<>("VendorJNI", RuntimeLoader.getDefaultExtractionRoot(), T265Camera.class);
        sLoader.loadLibrary();
        sLibraryLoaded = true;
    }

    public static enum PoseConfidence {
        Failed, Low, Medium, High,
    }

    public static class CameraUpdate {
        /**
         * The robot's pose in meters.
         */
        public final Pose2d pose;
        /**
         * The robot's velocity in meters/sec and radians/sec.
         */
        public final ChassisSpeeds velocity;
        public final PoseConfidence confidence;

        public CameraUpdate(Pose2d pose, ChassisSpeeds velocity, PoseConfidence confidence) {
            this.pose = pose;
            this.velocity = velocity;
            this.confidence = confidence;
        }
    }

    private long mNativeCameraObjectPointer = 0;
    private boolean mIsStarted = false;
    private Transform2d mRobotOffset;
    private Pose2d mOrigin = new Pose2d();
    private Pose2d mLastRecievedPose = new Pose2d();
    private Consumer<CameraUpdate> mPoseConsumer = null;

    /**
     * This method constructs a T265 camera and sets it up with the right info.
     * {@link T265Camera#start start} will not be called, you must call it
     * yourself.
     *
     * @param robotOffset        Offset of the center of the robot from the center
     *                           of the camera.
     * @param odometryCovariance Covariance of the odometry input when doing
     *                           sensor fusion (you probably want to tune this).
     */
    public T265Camera(Transform2d robotOffset, double odometryCovariance) {
        this(robotOffset, odometryCovariance, "");
    }

    /**
     * This method constructs a T265 camera and sets it up with the right info.
     * {@link T265Camera#start start} will not be called, you must call it
     * yourself.
     *
     * @param robotOffsetMeters  Offset of the center of the robot from the center
     *                           of the camera. Units are meters.
     * @param odometryCovariance Covariance of the odometry input when doing
     *                           sensor fusion (you probably want to tune this)
     * @param relocMapPath       path (including filename) to a relocalization map
     *                           to load.
     */
    public T265Camera(Transform2d robotOffsetMeters, double odometryCovariance, String relocMapPath) {
        mNativeCameraObjectPointer = newCamera(relocMapPath);
        setOdometryInfo((float) robotOffsetMeters.getTranslation().getX(),
                (float) robotOffsetMeters.getTranslation().getY(),
                (float) robotOffsetMeters.getRotation().getRadians(), odometryCovariance);
        mRobotOffset = robotOffsetMeters;
    }

    /**
     * This allows the user-provided pose receive callback to receive data.
     * This will also reset the camera's pose to (0, 0) at 0 degrees.
     * <p>
     * This will not restart the camera following exportRelocalizationMap. You will
     * have to call {@link T265Camera#free()} and make a new {@link T265Camera}.
     * This is related to what appears to be a bug in librealsense.
     *
     * @param poseConsumer A method to be called every time we receive a pose from
     *                     <i>from a different thread</i>! You must synchronize
     *                     memory access across threads!
     *                     <p>
     *                     Received poses are in meters.
     */
    public synchronized void start(Consumer<CameraUpdate> poseConsumer) {
        if (mIsStarted)
            throw new RuntimeException("T265 camera is already started");
        mPoseConsumer = poseConsumer;
        mIsStarted = true;
    }

    /**
     * This allows the callback to receivez data, but it does not internally stop the
     * camera.
     */
    public synchronized void stop() {
        mIsStarted = false;
    }

    /**
     * Exports a binary relocalization map file to the given path.
     * This will stop the camera. Because of a librealsense bug the camera isn't
     * restarted after you call this method. TODO: Fix that.
     *
     * @param path Path (with filename) to export to
     */
    public native void exportRelocalizationMap(String path);

    /**
     * Sends robot velocity as computed from wheel encoders.
     *
     * @param velocity The robot's translational velocity in meters/sec.
     */
    public void sendOdometry(Twist2d velocity) {
        Pose2d transVel = new Pose2d().exp(velocity);
        // Only 1 odometry sensor is supported for now (index 0)
        sendOdometryRaw(0, (float) transVel.getTranslation().getX(),
                (float) transVel.getTranslation().getY());
    }

    /**
     * This zeroes the camera pose to the provided new pose.
     *
     * @param newPose The pose the camera should be zeroed to.
     */
    public synchronized void setPose(Pose2d newPose) {
        mOrigin = newPose;
    }

    /**
     * This will free the underlying native objects. You probably don't want to use
     * this; on program shutdown the native code will gracefully stop and delete any
     * remaining objects.
     */
    public native void free();

    private native void setOdometryInfo(float robotOffsetX, float robotOffsetY,
                                        float robotOffsetRads, double measurementCovariance);

    private native void sendOdometryRaw(int sensorIndex, float xVel, float yVel);

    private native long newCamera(String mapPath);

    private static native void cleanup();

    private synchronized void consumePoseUpdate(float x, float y, float radians, float dx, float dy,
                                                float dtheta, int confOrdinal) {
        // First we apply an offset to go from the camera coordinate system to the
        // robot coordinate system with an origin at the center of the robot. This
        // is not a directional transformation.
        // Then we transform the pose our camera is giving us so that it reports is
        // the robot's pose, not the camera's. This is a directional transformation.
        final Pose2d currentPose = new Pose2d(x - mRobotOffset.getTranslation().getX(),
                y - mRobotOffset.getTranslation().getY(), new Rotation2d(radians))
                .transformBy(mRobotOffset);

        mLastRecievedPose = currentPose;

        if (!mIsStarted)
            return;

        // See
        // https://github.com/IntelRealSense/librealsense/blob/7f2ba0de8769620fd672f7b44101f0758e7e2fb3/include/librealsense2/h/rs_types.h#L115
        // for ordinals
        PoseConfidence confidence;
        switch (confOrdinal) {
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
                throw new RuntimeException(
                        "Unknown confidence ordinal \"" + confOrdinal + "\" passed from native code");
        }

        final Pose2d transformedPose = mOrigin.transformBy(new Transform2d(currentPose.getTranslation(), currentPose.getRotation()));

        mPoseConsumer.accept(new CameraUpdate(transformedPose,
                new ChassisSpeeds(dx, dy, dtheta), confidence));
    }

    /**
     * Thrown if something goes wrong in the native code
     */
    public static class CameraJNIException extends RuntimeException {

        // This must be static _and_ have this constructor if you want it to be
        // thrown from native code
        public CameraJNIException(String message) {
            super(message);
        }
    }
}
