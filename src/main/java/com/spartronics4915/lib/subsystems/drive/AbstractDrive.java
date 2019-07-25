package com.spartronics4915.lib.subsystems.drive;

import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;
import com.spartronics4915.lib.subsystems.Subsystem;

public abstract class AbstractDrive extends Subsystem
{
    // TODO: Come up with a sane way to do things like velocity or open loop or path following
    // TODO: Also maybe make the minimum path following subset of this into its own abstract class like FalconLib?

    protected final SpartronicsMotor mLeftMotor, mRightMotor;
    protected final SpartronicsIMU mIMU;

    protected Rotation2d mIMUOffset = Rotation2d.identity();

    private final double mWheelDiameterInches;
    private final double mNativeUnitsPerRotation;
    private final double mTrackWidthInches;

    protected AbstractDrive(SpartronicsMotor leftMotor, SpartronicsMotor rightMotor, SpartronicsIMU imu,
            double wheelDiameterInches, double nativeUnitsPerRotation, double trackWidthInches)
    {
        mLeftMotor = leftMotor;
        mRightMotor = rightMotor;
        mIMU = imu;

        mWheelDiameterInches = wheelDiameterInches;
        mNativeUnitsPerRotation = nativeUnitsPerRotation;
        mTrackWidthInches = trackWidthInches;
    }

    private double rotationsToInches(double rotations)
    {
        return rotations * (mWheelDiameterInches * Math.PI);
    }

    private double inchesToRotations(double inches)
    {
        return inches / (mWheelDiameterInches * Math.PI);
    }

    private double inchesPerSecondToNativeUnitsPer100ms(double ips)
    {
        return ((ips * mNativeUnitsPerRotation) / (mWheelDiameterInches * Math.PI)) / 10;
    }

    private double nativeUnitsPer100msToInchesPerSecond(double nu)
    {
        return (nu / mNativeUnitsPerRotation) * 10 * (mWheelDiameterInches * Math.PI);
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
        mIMUOffset = getIMUHeading().rotateBy(heading.inverse());
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

    public double getLeftDistanceInches()
    {
        return rotationsToInches(mLeftMotor.getEncoder().getPosition() / mNativeUnitsPerRotation);
    }

    public double getRightDistanceInches()
    {
        return rotationsToInches(mRightMotor.getEncoder().getPosition() / mNativeUnitsPerRotation);
    }

    public double getLeftVelocityInchesPerSec()
    {
        return rotationsToInches(mLeftMotor.getEncoder().getVelocity() * 10 / mNativeUnitsPerRotation);
    }

    public double getRightVelocityInchesPerSec()
    {
        return rotationsToInches(mRightMotor.getEncoder().getVelocity() * 10 / mNativeUnitsPerRotation);
    }

    public double getRobotLinearVelocityInchesPerSec()
    {
        return (getLeftVelocityInchesPerSec() + getRightVelocityInchesPerSec()) / 2.0;
    }

    public double getRobotAngularVelocityRadiansPerSec()
    {
        return (getLeftVelocityInchesPerSec() - getRightVelocityInchesPerSec()) / mTrackWidthInches;
    }

    public void arcadeDrive(double linearPercent, double rotationPercent)
    {
        double maxInput = Math.copySign(Math.max(Math.abs(linearPercent), Math.abs(rotationPercent)), linearPercent);

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
        mLeftMotor.setDutyCycle(leftPercent);
        mRightMotor.setDutyCycle(rightPercent);
    }
}
