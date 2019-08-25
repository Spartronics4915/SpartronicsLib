package com.spartronics4915.lib.subsystems.drive;

import java.util.Arrays;

import com.spartronics4915.lib.util.Logger;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class FeedRemoteCharacterization extends Command {

    private final AbstractDrive mDrive;
    private final NetworkTableEntry mAutoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
    private final NetworkTableEntry mTelemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
    /** Meters */
    private final double mWheelCircumference;

    private Number[] mOutputArray = new Number[10];
    private double mLeftInitialPosition = 0.0;
    private double mRightInitialPosition = 0.0;

    /**
     * This feeds this Python script:
     * https://github.com/robotpy/robot-characterization
     */
    public FeedRemoteCharacterization(AbstractDrive drive, double wheelDiameterMeters) {
        NetworkTableInstance.getDefault().setUpdateRate(0.010);
        
        mDrive = drive;
        mWheelCircumference = wheelDiameterMeters * Math.PI;
    }

    @Override
    public boolean isFinished() {
        // This must be manually stopped by the user via the Disable button
        return false;
    }

    @Override
    public void execute() {
        double now = Timer.getFPGATimestamp();

        // radians and radians/s
        double leftPosition = (mDrive.getLeftMotor().getEncoder().getPosition() - mLeftInitialPosition) / mWheelCircumference * (2 * Math.PI);
        double leftVelocity = mDrive.getLeftMotor().getEncoder().getVelocity() / mWheelCircumference * (2 * Math.PI);

        // radians and raians/s
        double rightPosition = (mDrive.getRightMotor().getEncoder().getPosition() - mRightInitialPosition) / mWheelCircumference * (2 * Math.PI);
        double rightVelocity = mDrive.getLeftMotor().getEncoder().getVelocity() / mWheelCircumference * (2 * Math.PI);

        // volts
        double battery = RobotController.getBatteryVoltage();

        // volts
        double leftMotorVolts = mDrive.getLeftMotor().getVoltageOutput();
        double rightMotorVolts = mDrive.getRightMotor().getVoltageOutput();

        // percent output on [-1 1]
        double autospeed = mAutoSpeedEntry.getDouble(0.0);

        mDrive.tankDrive(autospeed, autospeed);

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
    public void initialize() {
        mLeftInitialPosition = mDrive.getLeftMotor().getEncoder().getPosition();
        mRightInitialPosition = mDrive.getRightMotor().getEncoder().getPosition();
    }

}