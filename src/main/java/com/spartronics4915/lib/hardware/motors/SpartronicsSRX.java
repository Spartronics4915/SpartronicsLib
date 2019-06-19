package com.spartronics4915.lib.hardware.motors;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.spartronics4915.lib.subsystems.drive.SpartronicsEncoder;
import com.spartronics4915.lib.subsystems.drive.SpartronicsMotor;

public class SpartronicsSRX implements SpartronicsMotor
{

    // TODO: Sensor "model" like FalconLib? We could avoid Native Units

    private static final int kVelocitySlotIdx = 0;
    private static final int kPositionSlotIdx = 1;

    private TalonSRX mTalonSRX;
    private SpartronicsSRXEncoder mEncoder;

    private boolean mBrakeMode = false;
    private double mVoltageCompSaturation = 12.0;
    private double mMotionProfileCruiseVelocity = 0.0;
    private double mMotionProfileAcceleration = 0.0;
    private boolean mUseMotionProfileForPosition = false;

    private ControlMode mLastControlMode = null;

    public class SpartronicsSRXEncoder implements SpartronicsEncoder
    {

        @Override
        public double getVelocity()
        {
            return mTalonSRX.getSelectedSensorVelocity();
        }

        @Override
        public double getPosition()
        {
            return mTalonSRX.getSelectedSensorPosition();
        }

        @Override
        public void setPhase(boolean isReversed)
        {
            mTalonSRX.setSensorPhase(isReversed);
        }
    }

    public SpartronicsSRX(int deviceNumber)
    {
        mTalonSRX = new TalonSRX(deviceNumber);
        mEncoder = new SpartronicsSRXEncoder();
    }

    @Override
    public SpartronicsEncoder getEncoder()
    {
        ErrorCode err = mTalonSRX.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        return err == ErrorCode.OK ? mEncoder : null;
    }

    @Override
    public double getVoltageOutput()
    {
        return mTalonSRX.getMotorOutputVoltage();
    }

    @Override
    public boolean getOutputInverted()
    {
        return mTalonSRX.getInverted();
    }

    @Override
    public void setOutputInverted(boolean inverted)
    {
        mTalonSRX.setInverted(inverted);
    }

    @Override
    public boolean getBrakeMode()
    {
        return mBrakeMode;
    }

    @Override
    public void setBrakeMode(boolean mode)
    {
        mBrakeMode = mode;
        mTalonSRX.setNeutralMode(mode ? NeutralMode.Brake : NeutralMode.Coast);
    }

    @Override
    public double getVoltageCompSaturation()
    {
        return mVoltageCompSaturation;
    }

    @Override
    public void setVoltageCompSaturation(double voltage)
    {
        mVoltageCompSaturation = voltage;
        mTalonSRX.configVoltageCompSaturation(mVoltageCompSaturation);
        mTalonSRX.enableVoltageCompensation(true);
    }

    @Override
    public double getMotionProfileCruiseVelocity()
    {
        return mMotionProfileCruiseVelocity;
    }

    @Override
    public void setMotionProfileCruiseVelocity(double velocity)
    {
        mMotionProfileCruiseVelocity = velocity;
        mTalonSRX.configMotionCruiseVelocity((int) mMotionProfileCruiseVelocity);
    }

    @Override
    public double getMotionProfileMaxAcceleration()
    {
        return mMotionProfileAcceleration;
    }

    @Override
    public void setMotionProfileMaxAcceleration(double acceleration)
    {
        mMotionProfileAcceleration = acceleration;
        mTalonSRX.configMotionAcceleration((int) acceleration);
    }

    @Override
    public void setUseMotionProfileForPosition(boolean useMotionProfile)
    {
        mUseMotionProfileForPosition = useMotionProfile;
    }

    @Override
    public void setDutyCycle(double dutyCycle, double arbitraryFeedForward)
    {
        if (mLastControlMode != ControlMode.PercentOutput)
        {
            mLastControlMode = ControlMode.PercentOutput;
        }
        mTalonSRX.set(ControlMode.PercentOutput, dutyCycle, DemandType.ArbitraryFeedForward, arbitraryFeedForward);
    }

    @Override
    public void setDutyCycle(double dutyCycle)
    {
        setDutyCycle(dutyCycle, 0.0);
    }

    @Override
    public void setVelocity(double velocity, double arbitraryFeedForward)
    {
        if (mLastControlMode != ControlMode.Velocity)
        {
            mTalonSRX.selectProfileSlot(kVelocitySlotIdx, 0);
            mLastControlMode = ControlMode.Velocity;
        }
        mTalonSRX.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, arbitraryFeedForward);
    }

    @Override
    public void setVelocityGains(double kP, double kI, double kD, double kF)
    {
        mTalonSRX.config_kP(kVelocitySlotIdx, kP);
        mTalonSRX.config_kI(kVelocitySlotIdx, kI);
        mTalonSRX.config_kD(kVelocitySlotIdx, kD);
        mTalonSRX.config_kF(kVelocitySlotIdx, kF);
    }

    @Override
    public void setPosition(double position)
    {
        if (mLastControlMode != ControlMode.Position)
        {
            mTalonSRX.selectProfileSlot(kPositionSlotIdx, 0);
            mLastControlMode = ControlMode.Position;
        }
        mTalonSRX.selectProfileSlot(kPositionSlotIdx, 0);
        mTalonSRX.set(mUseMotionProfileForPosition ? ControlMode.MotionMagic : ControlMode.Position, position);
    }

    @Override
    public void setPositionGains(double kP, double kI, double kD, double kF)
    {
        mTalonSRX.config_kP(kPositionSlotIdx, kP);
        mTalonSRX.config_kI(kPositionSlotIdx, kI);
        mTalonSRX.config_kD(kPositionSlotIdx, kD);
        mTalonSRX.config_kF(kPositionSlotIdx, kF);
    }

}
