package com.spartronics4915.lib.subsystems.drive;

import com.spartronics4915.lib.hardware.motors.SpartronicsMotor;
import com.spartronics4915.lib.hardware.motors.SpartronicsSimulatedMotor;
import com.spartronics4915.lib.hardware.sensors.SpartronicsIMU;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;

public abstract class AbstractDrive extends SpartronicsSubsystem
{
    protected final SpartronicsMotor mLeftMotor, mRightMotor;
    protected final SpartronicsIMU mIMU;

    protected Rotation2d mIMUOffset = new Rotation2d();

    /**
     * This constructor will set up everything you need. It's protected to allow for
     * a singleton drivetrain.
     */
    protected AbstractDrive(SpartronicsMotor leftMotor, SpartronicsMotor rightMotor,
        SpartronicsIMU imu)
    {
        if (leftMotor.hadStartupError() || rightMotor.hadStartupError())
        {
            mLeftMotor = new SpartronicsSimulatedMotor(leftMotor.getDeviceNumber(), leftMotor.getFollower().getDeviceNumber());
            mRightMotor = new SpartronicsSimulatedMotor(rightMotor.getDeviceNumber(), rightMotor.getFollower().getDeviceNumber());
            mIMU = new SpartronicsIMU()
            {
                @Override
                public Rotation2d getYaw()
                {
                    return new Rotation2d();
                }
            };

            logInitialized(false);
        }
        else
        {
            mLeftMotor = leftMotor;
            mRightMotor = rightMotor;
            mIMU = imu;

            logInitialized(true);
        }
    }

    /**
     * This sets the heading of the IMU, but this should almost exclusively be
     * called by RobotStateEstimator, because that is the single source of truth for
     * robot heading.
     *
     * @param heading Heading to set the IMU to.
     */
    public void setIMUHeading(Rotation2d heading)
    {
        mIMUOffset = mIMU.getYaw().minus(heading);
    }

    /**
     * This gets the heading of the IMU, but this should almost exclusively be
     * called by RobotStateMap, which is the single source of truth for all matters
     * of robot pose (including heading).
     *
     * @return Heading of the IMU.
     */
    public Rotation2d getIMUHeading()
    {
        return mIMU.getYaw().rotateBy(mIMUOffset);
    }

    public double getTurretAngle()
    {
        return 0;
    }

    public void arcadeDrive(double linearPercent, double rotationPercent)
    {
        double maxInput = Math.copySign(Math.max(Math.abs(linearPercent), 
                                        Math.abs(rotationPercent)), linearPercent);

        double leftMotorOutput, rightMotorOutput;

        if (linearPercent >= 0.0)
        {
            // First quadrant, else second quadrant
            if (rotationPercent >= 0.0)
            {
                leftMotorOutput = maxInput;
                rightMotorOutput = linearPercent - rotationPercent;
            }
            else
            {
                leftMotorOutput = linearPercent + rotationPercent;
                rightMotorOutput = maxInput;
            }
        }
        else
        {
            // Third quadrant, else fourth quadrant
            if (rotationPercent >= 0.0)
            {
                leftMotorOutput = linearPercent + rotationPercent;
                rightMotorOutput = maxInput;
            }
            else
            {
                leftMotorOutput = maxInput;
                rightMotorOutput = linearPercent - rotationPercent;
            }
        }

        tankDrive(leftMotorOutput, rightMotorOutput);
    }

    public void tankDrive(double leftPercent, double rightPercent)
    {
        mLeftMotor.setPercentOutput(leftPercent);
        mRightMotor.setPercentOutput(rightPercent);
    }

    public SpartronicsMotor getLeftMotor()
    {
        return mLeftMotor;
    }

    public SpartronicsMotor getRightMotor()
    {
        return mRightMotor;
    }

    @Override
    public void periodic()
    {
        dashboardPutNumber("imuHeading", mIMU.getYaw().getDegrees());
        dashboardPutNumber("imuHeadingAdjusted", getIMUHeading().getDegrees());

        dashboardPutNumber("leftSpeed", getLeftMotor().getEncoder().getVelocity());
        dashboardPutNumber("rightSpeed", getRightMotor().getEncoder().getVelocity());
    }
}
