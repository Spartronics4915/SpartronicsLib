package com.spartronics4915.lib.subsystems.estimator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;
import com.spartronics4915.lib.math.twodim.geometry.Twist2d;
import com.spartronics4915.lib.hardware.sensors.T265Camera;
import com.spartronics4915.lib.hardware.sensors.T265Camera.CameraUpdate;
import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;
import com.spartronics4915.lib.subsystems.drive.AbstractDrive;
import com.spartronics4915.lib.util.Kinematics;
import com.spartronics4915.lib.util.Units;

/**
 * This loop keeps track of robot state whenever the robot is enabled.
 * 
 * TODO: Split out to CameraRobotStateEstimator and DifferentialDriveRobotStateEstimator.
 * Also TODO: Remove all the velocity stuff from robot states.
 */
public class RobotStateEstimator extends SpartronicsSubsystem
{

    /**
     * The SLAM camera/encoder RobotStateMap objects represent two views of the
     * robot's current state (state is pose, velocity, and distance driven).
     * 
     * All length units are meters.
     */
    private RobotStateMap mEncoderStateMap = new RobotStateMap();
    private RobotStateMap mCameraStateMap = new RobotStateMap();

    private AbstractDrive mDrive;
    private Kinematics mKinematics;
    private T265Camera mSLAMCamera;

    /** Meters */
    private double mLeftPrevDist = 0.0, mRightPrevDist = 0.0;

    public RobotStateEstimator(AbstractDrive driveSubsystem, Kinematics kinematics, T265Camera slamra)
    {
        mDrive = driveSubsystem;
        mKinematics = kinematics;
        mSLAMCamera = slamra;

        resetRobotStateMaps();

        // Run this at 20 Hz
        new Notifier(this::run).startPeriodic(1 / 100.0);
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
        resetRobotStateMaps(new Pose2d());
    }

    public synchronized void resetRobotStateMaps(Pose2d pose)
    {
        double time = Timer.getFPGATimestamp();
        mEncoderStateMap.reset(time, pose);
        mCameraStateMap.reset(time, pose);

        mLeftPrevDist = mDrive.getLeftMotor().getEncoder().getPosition();
        mRightPrevDist = mDrive.getRightMotor().getEncoder().getPosition();

        mSLAMCamera.setPose(pose);
        // mDrive.setIMUHeading(pose.getRotation()); TODO: Put back when I get a real IMU
    }

    @Override
    public void periodic()
    {
        super.periodic();

        final RobotStateMap.State estate = mEncoderStateMap.getLatestState();
        Pose2d epose = estate.pose;
        SmartDashboard.putString("RobotState/encoderPose",
                Units.metersToInches(epose.getTranslation().getX()) +
                        " " + Units.metersToInches(epose.getTranslation().getY()) +
                        " " + epose.getRotation().getDegrees());
        SmartDashboard.putNumber("RobotState/encoderVelocity", estate.predictedVelocity.dx);

        final RobotStateMap.State cstate = getCameraRobotStateMap().getLatestState();
        Pose2d cpose = cstate.pose;
        SmartDashboard.putString("RobotState/pose",
                Units.metersToInches(cpose.getTranslation().getX()) +
                        " " + Units.metersToInches(cpose.getTranslation().getY()) +
                        " " + cpose.getRotation().getDegrees());
        SmartDashboard.putNumber("RobotState/velocity", cstate.predictedVelocity.dx);
    }

    public void stop()
    {
        mSLAMCamera.stop();
    }

    public void run()
    {
        if (DriverStation.getInstance().isDisabled())
            return;

        final RobotStateMap.State last = mEncoderStateMap.getLatestState();

        /*
         * There are two ways to measure current velocity:
         * Method 1, integrationVelocity
         * Look at the distance traveled since last measurement, consider
         * current gyro heading rather than our stored state
         * Divide by delta time to produce a velocity. Note that
         * 254's implementation doesn't include time computations explicitly.
         * In method 1, the implicit time is the time between samples which relates
         * to the looper time interval. Thus: leftDelta is measured in
         * meters/loopinterval. To the degree that the loop interval isn't a
         * constant the result will be noisy. OTH: we can interpret this
         * velocity as also a distance traveled since last loop.
         */
        final Twist2d iVal;
        synchronized (this) {
            final double leftDist = mDrive.getLeftMotor().getEncoder().getPosition();
            final double rightDist = mDrive.getRightMotor().getEncoder().getPosition();
            final double leftDelta = leftDist - mLeftPrevDist;
            final double rightDelta = rightDist - mRightPrevDist;
            final Rotation2d heading = mDrive.getIMUHeading();
            mLeftPrevDist = leftDist;
            mRightPrevDist = rightDist;
            iVal = mKinematics.forwardKinematics(last.pose.getRotation(), leftDelta, rightDelta, heading);
        }

        /*
         * Method 2, 'predictedVelocity'
         * Directly sample the current wheel velocities. Here, linear velocities
         * are measured in meters/sec. Since the integration step below expects
         * velocity to be measured in meters/loopinterval, this version of velocity
         * can't be used directly. Moreover, the velocity we obtain from the wheel
         * encoders is integrated over a different time interval than one
         * loop-interval. It's not clear which estimation technique would deliver
         * a better result. For visualization purposes velocity2 (in meters/sec)
         * is in human-readable form. Also of note, this variant doesn't
         * include the gyro heading in its calculation.
         */
        final Twist2d pVal = mKinematics.forwardKinematics(
                mDrive.getLeftMotor().getEncoder().getVelocity(),
                mDrive.getRightMotor().getEncoder().getVelocity());

        /*
         * integrateForward: given a last state and a current velocity,
         * estimate a new state (P2 = P1 + dPdt * dt)
         */
        final Pose2d nextP = mKinematics.integrateForwardKinematics(last.pose, iVal);

        /* record the new state estimate */
        mEncoderStateMap.addObservations(Timer.getFPGATimestamp(), nextP, iVal, pVal);

        // We convert meters/loopinterval and radians/loopinterval to meters/sec and radians/sec
        final double loopintervalToSeconds = 1 / (Timer.getFPGATimestamp() - last.timestamp);
        final Twist2d normalizedIVal = iVal.scaled(loopintervalToSeconds);

        mSLAMCamera.sendOdometry(pVal);
    }

    public void enable()
    {
        // Callback is called from a different thread... We avoid data races because RobotSteteMap is thread-safe
        mSLAMCamera.stop();
        mSLAMCamera.start((CameraUpdate update) ->
        {
            mCameraStateMap.addObservations(Timer.getFPGATimestamp(), update.pose, update.velocity, new Twist2d());
            SmartDashboard.putString("RobotState/cameraConfidence", update.confidence.toString());
        });
    }
}
