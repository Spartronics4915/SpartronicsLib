package com.spartronics4915.lib.loops.estimator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.spartronics4915.lib.math.geometry.Pose2d;
import com.spartronics4915.lib.math.geometry.Rotation2d;
import com.spartronics4915.lib.math.geometry.Twist2d;
import com.spartronics4915.lib.sensors.T265Camera;
import com.spartronics4915.lib.statemachine.Loop;
import com.spartronics4915.lib.subsystems.Subsystem;
import com.spartronics4915.lib.subsystems.drive.Drive;
import com.spartronics4915.lib.util.Kinematics;

/**
 * This loop keeps track of robot state whenever the robot is enabled.
 */
public class RobotStateEstimator implements Loop
{

    /**
     * The SLAM camera/encoder RobotStateMap objects represent two views of the
     * robot's current state (state is pose, velocity, and distance driven).
     */
    private RobotStateMap mEncoderStateMap = new RobotStateMap();
    private RobotStateMap mCameraStateMap = new RobotStateMap();

    private Drive mDrive;
    private Kinematics mKinematics;
    private T265Camera mSLAMCamera;

    private double mLeftPrevDist = 0.0;
    private double mRightPrevDist = 0.0;

    private static final Pose2d kZeroPose = Pose2d.identity();

    public RobotStateEstimator(Drive driveSubsystem, Kinematics kinematics, T265Camera slamra)
    {
        mDrive = driveSubsystem;
        mKinematics = kinematics;
        mSLAMCamera = slamra;
    }

    public RobotStateMap getEncoderRobotStateMap()
    {
        return mEncoderStateMap;
    }

    public RobotStateMap getCameraRobotStateMap()
    {
        return mCameraStateMap;
    }

    public void resetRobotStateMaps()
    {
        resetRobotStateMaps(kZeroPose);
    }

    public void resetRobotStateMaps(Pose2d pose)
    {
        double time = Timer.getFPGATimestamp();
        mEncoderStateMap.reset(time, pose);
        mCameraStateMap.reset(time, pose);
        mDrive.setIMUHeading(pose.getRotation());
    }

    public void outputTelemetry()
    {
        final RobotStateMap.State estate = mEncoderStateMap.getLatestState();
        Pose2d epose = estate.pose;
        SmartDashboard.putString("RobotState/pose",
                epose.getTranslation().x() +
                        " " + epose.getTranslation().y() +
                        " " + epose.getRotation().getDegrees());
        Twist2d pVel = estate.predictedVelocity;
        SmartDashboard.putNumber("RobotState/velocity", pVel.dx);
    }

    @Override
    public void stop()
    {
        mSLAMCamera.stop();
    }

    @Override
    public void run(boolean shouldReset)
    {
        if (shouldReset)
        {
            mLeftPrevDist = mDrive.getLeftDistanceInches();
            mRightPrevDist = mDrive.getRightDistanceInches();

            // Called from a different thread... We avoid data races because RobotSteteMap is thread-safe
            mSLAMCamera.start((Pose2d pose, T265Camera.PoseConfidence conf) ->
            {
                // TODO: Fix velocities
                mCameraStateMap.addObservations(Timer.getFPGATimestamp(), pose, Twist2d.identity(), Twist2d.identity());
            });
            mSLAMCamera.setPose(Pose2d.identity());
        }

        final RobotStateMap.State last = mEncoderStateMap.getLatestState();
        final Pose2d lastPose = last.pose;

        /*
         * There are two ways to measure current velocity:
         * Method 1, integrationVelocity
         * Look at the distance traveled since last measurement, consider
         * current gyro heading rather than our stored state
         * Divide by delta time to produce a velocity. Note that
         * 254's implementation doesn't include time computations explicitly.
         * In method 1, the implicit time is the time between samples which relates
         * to the looper time interval. Thus: leftDelta is measured in
         * inches/loopinterval. To the degree that the loop interval isn't a
         * constant the result will be noisy. OTH: we can interpret this
         * velocity as also a distance traveled since last loop.
         */
        final double leftDist = mDrive.getLeftDistanceInches();
        final double rightDist = mDrive.getRightDistanceInches();
        final double leftDelta = leftDist - mLeftPrevDist;
        final double rightDelta = rightDist - mRightPrevDist;
        final Rotation2d heading = mDrive.getIMUHeading();
        mLeftPrevDist = leftDist;
        mRightPrevDist = rightDist;
        final Twist2d iVal = mKinematics.forwardKinematics(
                lastPose.getRotation(),
                leftDelta, rightDelta, heading);

        /*
         * Method 2, 'predictedVelocity'
         * Directly sample the current wheel velocities. Here, linear velocities
         * are measured in inches/sec. Since the integration step below expects
         * velocity to be measured in inches/loopinterval, this version of velocity
         * can't be used directly. Moreover, the velocity we obtain from the wheel
         * encoders is integrated over a different time interval than one
         * loop-interval. It's not clear which estimation technique would deliver
         * a better result. For visualization purposes velocity2 (in inches/sec)
         * is in human-readable form. Also of note, this variant doesn't
         * include the gyro heading in its calculation.
         */
        final Twist2d pVal = mKinematics.forwardKinematics(
                mDrive.getLeftVelocityInchesPerSec(),
                mDrive.getRightVelocityInchesPerSec());

        /*
         * integrateForward: given a last state and a current velocity,
         * estimate a new state (P2 = P1 + dPdt * dt)
         */
        final Pose2d nextP = mKinematics.integrateForwardKinematics(last.pose, iVal);

        /* record the new state estimate */
        mEncoderStateMap.addObservations(Timer.getFPGATimestamp(), nextP, iVal, pVal);

        outputTelemetry();
    }
}