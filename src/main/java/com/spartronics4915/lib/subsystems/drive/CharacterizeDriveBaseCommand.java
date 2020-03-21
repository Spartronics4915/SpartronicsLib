package com.spartronics4915.lib.subsystems.drive;

import java.util.Set;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class CharacterizeDriveBaseCommand implements Command
{
    private final AbstractDrive mDrive;
    private final NetworkTableEntry mAutoSpeedEntry = NetworkTableInstance.getDefault()
        .getEntry("/robot/autospeed");
    private final NetworkTableEntry mTelemetryEntry = NetworkTableInstance.getDefault()
        .getEntry("/robot/telemetry");
    private final NetworkTableEntry rotateEntry = NetworkTableInstance.getDefault()
        .getEntry("/robot/rotate");

    /** Meters */
    private final double mWheelCircumference;

    private Number[] mOutputArray = new Number[10];
    private double mLeftInitialPosition = 0.0;
    private double mRightInitialPosition = 0.0;

    /**
     * This feeds this Python script:
     * https://github.com/robotpy/robot-characterization
     */
    public CharacterizeDriveBaseCommand(AbstractDrive drive, double wheelDiameterMeters)
    {
        NetworkTableInstance.getDefault().setUpdateRate(0.010);

        mDrive = drive;
        mWheelCircumference = wheelDiameterMeters * Math.PI;
    }

    @Override
    public boolean isFinished()
    {
        // This must be manually stopped by the user via the Disable button
        return false;
    }

    @Override
    public void execute()
    {
        double now = Timer.getFPGATimestamp();

        // radians and radians/s
        double leftPosition = (mDrive.getLeftMotor().getEncoder().getPosition()
            - mLeftInitialPosition) / mWheelCircumference * (2 * Math.PI);
        double leftVelocity = mDrive.getLeftMotor().getEncoder().getVelocity() / mWheelCircumference
            * (2 * Math.PI);

        // radians and radians/s
        double rightPosition = (mDrive.getRightMotor().getEncoder().getPosition()
            - mRightInitialPosition) / mWheelCircumference * (2 * Math.PI);
        double rightVelocity = mDrive.getLeftMotor().getEncoder().getVelocity()
            / mWheelCircumference * (2 * Math.PI);

        // volts
        double battery = RobotController.getBatteryVoltage();

        // volts
        double leftMotorVolts = mDrive.getLeftMotor().getVoltageOutput();
        double rightMotorVolts = mDrive.getRightMotor().getVoltageOutput();

        // percent output on [-1 1]
        double autospeed = mAutoSpeedEntry.getDouble(0.0);

        mDrive.tankDrive((rotateEntry.getBoolean(false) ? -1 : 1) * autospeed, autospeed);

        // send telemetry data array back to NT
        mOutputArray[0] = now;
        mOutputArray[1] = battery;
        mOutputArray[2] = autospeed;
        mOutputArray[3] = leftMotorVolts;
        mOutputArray[4] = rightMotorVolts;
        mOutputArray[5] = leftPosition;
        mOutputArray[6] = rightPosition;
        mOutputArray[7] = leftVelocity;
        mOutputArray[8] = rightVelocity;
        mOutputArray[9] = mDrive.getIMUHeading().getRadians();

        mTelemetryEntry.setNumberArray(mOutputArray);
    }

    @Override
    public void initialize()
    {
        mLeftInitialPosition = mDrive.getLeftMotor().getEncoder().getPosition();
        mRightInitialPosition = mDrive.getRightMotor().getEncoder().getPosition();
    }

    @Override
    public Set<Subsystem> getRequirements()
    {
        return Set.of(mDrive);
    }
}
