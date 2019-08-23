package com.spartronics4915.lib.subsystems.drive;

import com.spartronics4915.lib.hardware.motors.SpartronicsMotor;
import com.spartronics4915.lib.hardware.sensors.SpartronicsIMU;
import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;
import com.spartronics4915.lib.math.twodim.physics.DifferentialDrive;
import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;

public abstract class AbstractDrive extends SpartronicsSubsystem implements DifferentialTrackerDriveBase
{
    protected final SpartronicsMotor mLeftMotor, mRightMotor;
    protected final SpartronicsIMU mIMU;
    protected final DifferentialDrive mDifferentialDrive;

    protected Rotation2d mIMUOffset = new Rotation2d();

    /**
     * This constructor will set up everything you need. It's protected to allow for
     * a singleton drivetrain.
     */
    protected AbstractDrive(
        SpartronicsMotor leftMotor,
        SpartronicsMotor rightMotor,
        SpartronicsIMU imu,
        DifferentialDrive differentialDrive
    )
    {
        mLeftMotor = leftMotor;
        mRightMotor = rightMotor;
        mIMU = imu;
        mDifferentialDrive = differentialDrive;
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

    /**
     * @return The displacement of the left wheel, in meters
     */
    public double getLeftPosition()
    {
        return mLeftMotor.getEncoder().getPosition();
    }

    /**
     * @return The displacement of the right wheel, in meters
     */
    public double getRightPosition()
    {
        return mRightMotor.getEncoder().getPosition();
    }

    /**
     * @return The left wheel's velocity in meters/second
     */
    public double getLeftVelocity()
    {
        return mLeftMotor.getEncoder().getVelocity();
    }

    /**
     * @return The right wheel's velocity in meters/second
     */
    public double getRightVelocity()
    {
        return mRightMotor.getEncoder().getVelocity();
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

    @Override
    public SpartronicsMotor getLeftMotor() {
        return mLeftMotor;
    }

    @Override
    public SpartronicsMotor getRightMotor() {
        return mRightMotor;
    }

    @Override
    public DifferentialDrive getDifferentialDrive() {
        return mDifferentialDrive;
    }
}
