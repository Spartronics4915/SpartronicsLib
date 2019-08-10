package com.spartronics4915.lib.hardware.motors;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class SpartronicsSRX implements SpartronicsMotor
{

    private static final int kVelocitySlotIdx = 0;
    private static final int kPositionSlotIdx = 1;

    private TalonSRX mTalonSRX;
    private SpartronicsSRXEncoder mEncoder;
    private SensorModel mSensorModel;

    private boolean mBrakeMode = false;
    /** Volts */
    private double mVoltageCompSaturation = 12.0;
    /** Native units/sec, converted to meters on get and set */
    private double mMotionProfileCruiseVelocity = 0.0;
    /** Native units/sec^2, converted to meters on get and set */
    private double mMotionProfileAcceleration = 0.0;
    private boolean mUseMotionProfileForPosition = false;

    private ControlMode mLastControlMode = null;

    public class SpartronicsSRXEncoder implements SpartronicsEncoder
    {

        @Override
        public double getVelocity()
        {
            return mSensorModel.toMeters(mTalonSRX.getSelectedSensorVelocity());
        }

        @Override
        public double getPosition()
        {
            return mSensorModel.toMeters(mTalonSRX.getSelectedSensorPosition());
        }

        @Override
        public void setPhase(boolean isReversed)
        {
            mTalonSRX.setSensorPhase(isReversed);
        }
    }

    public SpartronicsSRX(int deviceNumber, SensorModel sensorModel)
    {
        this(new TalonSRX(deviceNumber), sensorModel);
    }

    public SpartronicsSRX(TalonSRX talon, SensorModel sensorModel)
    {
        mTalonSRX = talon;
        mEncoder = new SpartronicsSRXEncoder();
        mSensorModel = sensorModel;

        mTalonSRX.configFactoryDefault();
        mTalonSRX.configVoltageCompSaturation(mVoltageCompSaturation);
        mTalonSRX.enableVoltageCompensation(true);
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
        return mSensorModel.toMeters(mMotionProfileCruiseVelocity);
    }

    @Override
    public void setMotionProfileCruiseVelocity(double velocityMetersPerSecond)
    {
        mMotionProfileCruiseVelocity = mSensorModel.toNativeUnits(velocityMetersPerSecond);
        mTalonSRX.configMotionCruiseVelocity((int) mMotionProfileCruiseVelocity);
    }

    @Override
    public double getMotionProfileMaxAcceleration()
    {
        return mMotionProfileAcceleration;
    }

    @Override
    public void setMotionProfileMaxAcceleration(double accelerationMetersPerSecondSq)
    {
        mMotionProfileAcceleration = mSensorModel.toNativeUnits(accelerationMetersPerSecondSq);
        mTalonSRX.configMotionAcceleration((int) accelerationMetersPerSecondSq);
    }

    @Override
    public void setUseMotionProfileForPosition(boolean useMotionProfile)
    {
        mUseMotionProfileForPosition = useMotionProfile;
    }

    @Override
    public void setDutyCycle(double dutyCycle, double arbitraryFeedForwardVolts)
    {
        if (mLastControlMode != ControlMode.PercentOutput)
        {
            mLastControlMode = ControlMode.PercentOutput;
        }
        mTalonSRX.set(
            ControlMode.PercentOutput, dutyCycle,
            DemandType.ArbitraryFeedForward, arbitraryFeedForwardVolts / mVoltageCompSaturation
        );
    }

    @Override
    public void setDutyCycle(double dutyCycle)
    {
        setDutyCycle(dutyCycle, 0.0);
    }

    @Override
    public void setVelocity(double velocityMetersPerSecond, double arbitraryFeedForwardVolts)
    {
        if (mLastControlMode != ControlMode.Velocity)
        {
            mTalonSRX.selectProfileSlot(kVelocitySlotIdx, 0);
            mLastControlMode = ControlMode.Velocity;
        }

        double velocityNative = mSensorModel.toNativeUnits(velocityMetersPerSecond);
        mTalonSRX.set(
            ControlMode.Velocity, velocityNative,
            DemandType.ArbitraryFeedForward, arbitraryFeedForwardVolts / mVoltageCompSaturation
        );
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
    public void setPosition(double positionMeters)
    {
        if (mLastControlMode != ControlMode.Position)
        {
            mTalonSRX.selectProfileSlot(kPositionSlotIdx, 0);
            mLastControlMode = ControlMode.Position;
        }
        mTalonSRX.selectProfileSlot(kPositionSlotIdx, 0);

        positionMeters = mSensorModel.toNativeUnits(positionMeters);
        mTalonSRX.set(mUseMotionProfileForPosition ? ControlMode.MotionMagic : ControlMode.Position, positionMeters);
    }

    @Override
    public void setPositionGains(double kP, double kI, double kD, double kF)
    {
        mTalonSRX.config_kP(kPositionSlotIdx, kP);
        mTalonSRX.config_kI(kPositionSlotIdx, kI);
        mTalonSRX.config_kD(kPositionSlotIdx, kD);
        mTalonSRX.config_kF(kPositionSlotIdx, kF);
    }

    public void follow(SpartronicsSRX other)
    {
        mTalonSRX.follow(other.mTalonSRX);
    }

    @Override
    public SensorModel getSensorModel() {
        return mSensorModel;
    }

    @Override
    public void setVelocity(double velocityMetersPerSecond) {
        mTalonSRX.set(ControlMode.Velocity, mSensorModel.toNativeUnits(velocityMetersPerSecond));
    }

    @Override
    public void setNeutral() {
        mTalonSRX.set(ControlMode.Disabled, 0.0, DemandType.Neutral, 0.0);
    }

}
