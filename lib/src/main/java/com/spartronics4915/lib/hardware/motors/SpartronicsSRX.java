package com.spartronics4915.lib.hardware.motors;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.spartronics4915.lib.hardware.CANCounter;
import com.spartronics4915.lib.util.Logger;

import edu.wpi.first.wpilibj.RobotBase;

public class SpartronicsSRX implements SpartronicsMotor
{
    private static final int kVelocitySlotIdx = 0;
    private static final int kPositionSlotIdx = 1;

    // This conversion could go into the sensor model, but the per 100ms thing is
    // Talon-only, so it's not worth it.
    private static final double kMetersPer100msToMetersPerSecond = 10;
    private static final double kMetersPerSecondToMetersPer100ms = 1
        / kMetersPer100msToMetersPerSecond;

    private final TalonSRX mTalonSRX;
    private final TalonSRX mFollower;
    private final SpartronicsEncoder mEncoder;
    private final SensorModel mSensorModel;
    private final boolean mHadStartupError;

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
            return mSensorModel.toCustomUnits(mTalonSRX.getSelectedSensorVelocity())
                * kMetersPer100msToMetersPerSecond;
        }

        @Override
        public double getPosition()
        {
            return mSensorModel.toCustomUnits(mTalonSRX.getSelectedSensorPosition());
        }

        @Override
        public void setPhase(boolean isReversed)
        {
            mTalonSRX.setSensorPhase(isReversed);
        }

        @Override
        public boolean setPosition(double position)
        {
            mTalonSRX.getSensorCollection()
                .setQuadraturePosition((int) mSensorModel.toNativeUnits(position), 0);
            return true;
        }
    }

    public static SpartronicsMotor makeMotor(int deviceNumber, SensorModel sensorModel,
        FeedbackDevice feedbackDevice)
    {
        if (RobotBase.isSimulation())
        {
            return new SpartronicsSimulatedMotor(deviceNumber);
        }
        return new SpartronicsSRX(new TalonSRX(deviceNumber), sensorModel, feedbackDevice, null);
    }

    public static SpartronicsMotor makeMotor(int deviceNumber, SensorModel sensorModel)
    {
        return makeMotor(deviceNumber, sensorModel, FeedbackDevice.QuadEncoder);
    }

    public static SpartronicsMotor makeMotor(int deviceNumber)
    {
        return makeMotor(deviceNumber, SensorModel.fromMultiplier(1));
    }

    public static SpartronicsMotor makeMotor(int deviceNumber, SensorModel sensorModel,
        int followerDeviceNumber)
    {
        if (RobotBase.isSimulation())
        {
            return new SpartronicsSimulatedMotor(deviceNumber, followerDeviceNumber);
        }
        var master = new TalonSRX(deviceNumber);
        var follower = new TalonSRX(followerDeviceNumber);
        follower.follow(master);

        return new SpartronicsSRX(master, sensorModel, FeedbackDevice.QuadEncoder, follower);
    }

    private SpartronicsSRX(TalonSRX talon, SensorModel sensorModel, FeedbackDevice encoder,
        TalonSRX follower)
    {
        mTalonSRX = talon;
        mFollower = follower;
        mSensorModel = sensorModel;

        ErrorCode err = mTalonSRX.configSelectedFeedbackSensor(encoder, 0, 5);
        if (err != ErrorCode.OK)
        {
            Logger.error("TalonSRX on with ID " + mTalonSRX.getDeviceID()
                + " returned a non-OK error code on sensor configuration... Is the encoder plugged in?");
            mHadStartupError = true;
        }
        else
        {
            mHadStartupError = false;
        }
        CANCounter.addDevice(mHadStartupError);

        mEncoder = new SpartronicsSRXEncoder();

        mTalonSRX.configFactoryDefault();
        mTalonSRX.configVoltageCompSaturation(mVoltageCompSaturation);
        mTalonSRX.enableVoltageCompensation(true);
    }

    @Override
    public SpartronicsEncoder getEncoder()
    {
        return mEncoder;
    }

    @Override
    public boolean hadStartupError()
    {
        return false;//mHadStartupError; Change back when comp season is over!
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
        return mSensorModel.toCustomUnits(mMotionProfileCruiseVelocity)
            * kMetersPer100msToMetersPerSecond;
    }

    @Override
    public void setMotionProfileCruiseVelocity(double velocityMetersPerSecond)
    {
        mMotionProfileCruiseVelocity = mSensorModel
            .toNativeUnits(velocityMetersPerSecond * kMetersPerSecondToMetersPer100ms);
        mTalonSRX.configMotionCruiseVelocity((int) mMotionProfileCruiseVelocity);
    }

    @Override
    public double getMotionProfileMaxAcceleration()
    {
        return mSensorModel.toCustomUnits(mMotionProfileAcceleration)
            * kMetersPer100msToMetersPerSecond;
    }

    @Override
    public void setMotionProfileMaxAcceleration(double accelerationMetersPerSecondSq)
    {
        mMotionProfileAcceleration = mSensorModel
            .toNativeUnits(accelerationMetersPerSecondSq * kMetersPerSecondToMetersPer100ms);
        mTalonSRX.configMotionAcceleration((int) mMotionProfileAcceleration);
    }

    @Override
    public void setUseMotionProfileForPosition(boolean useMotionProfile)
    {
        mUseMotionProfileForPosition = useMotionProfile;
    }

    @Override
    public void setPercentOutput(double dutyCycle, double arbitraryFeedForwardVolts)
    {
        mLastControlMode = ControlMode.PercentOutput;
        mTalonSRX.set(ControlMode.PercentOutput, dutyCycle, DemandType.ArbitraryFeedForward,
            arbitraryFeedForwardVolts / mVoltageCompSaturation);
    }

    @Override
    public void setPercentOutput(double dutyCycle)
    {
        setPercentOutput(dutyCycle, 0.0);
    }

    @Override
    public void setVelocity(double velocityMetersPerSecond, double arbitraryFeedForwardVolts)
    {
        if (mLastControlMode != ControlMode.Velocity)
        {
            mTalonSRX.selectProfileSlot(kVelocitySlotIdx, 0);
            mLastControlMode = ControlMode.Velocity;
        }

        double velocityNative = mSensorModel
            .toNativeUnits(velocityMetersPerSecond * kMetersPerSecondToMetersPer100ms);
        mTalonSRX.set(ControlMode.Velocity, velocityNative, DemandType.ArbitraryFeedForward,
            arbitraryFeedForwardVolts / mVoltageCompSaturation);
    }

    @Override
    public void setVelocityGains(double kP, double kD)
    {
        setVelocityGains(kP, 0, kD, 0);
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

        double positionNative = mSensorModel.toNativeUnits(positionMeters);
        mTalonSRX.set(mUseMotionProfileForPosition ? ControlMode.MotionMagic : ControlMode.Position,
            positionNative);
    }

    @Override
    public void setPositionGains(double kP, double kD)
    {
        setPositionGains(kP, 0, kD, 0);
    }

    @Override
    public void setPositionGains(double kP, double kI, double kD, double kF)
    {
        mTalonSRX.config_kP(kPositionSlotIdx, kP);
        mTalonSRX.config_kI(kPositionSlotIdx, kI);
        mTalonSRX.config_kD(kPositionSlotIdx, kD);
        mTalonSRX.config_kF(kPositionSlotIdx, kF);
    }

    @Override
    public SensorModel getSensorModel()
    {
        return mSensorModel;
    }

    @Override
    public void setVelocity(double velocityMetersPerSecond)
    {
        setVelocity(velocityMetersPerSecond, 0.0);
    }

    @Override
    public void setNeutral()
    {
        mTalonSRX.set(ControlMode.Disabled, 0.0, DemandType.Neutral, 0.0);
    }

    @Override
    public double getOutputCurrent()
    {
        return mTalonSRX.getStatorCurrent();
    }

    @Override
    public SpartronicsMotor getFollower()
    {
        return new SpartronicsSRX(mFollower, mSensorModel, FeedbackDevice.None, null);
    }

    @Override
    public int getDeviceNumber()
    {
        return mTalonSRX.getDeviceID();
    }

    @Override
    public void setSoftLimits(double forwardLimitCustomUnits, double reverseLimitCustomUnits)
    {
        mTalonSRX.configForwardSoftLimitEnable(true);
        mTalonSRX.configReverseSoftLimitEnable(true);

        mTalonSRX.configForwardSoftLimitThreshold(
            (int) Math.round(mSensorModel.toNativeUnits(forwardLimitCustomUnits)));
        mTalonSRX.configReverseSoftLimitThreshold(
            (int) Math.round(mSensorModel.toNativeUnits(reverseLimitCustomUnits)));
    }

    @Override
    public void setSupplyCurrentLimit(int limitAmps, double maxTimeAtLimit)
    {
        mTalonSRX.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, limitAmps, limitAmps, maxTimeAtLimit));
    }

}
