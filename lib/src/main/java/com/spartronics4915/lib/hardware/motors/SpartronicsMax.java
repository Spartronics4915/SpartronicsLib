package com.spartronics4915.lib.hardware.motors;

import com.revrobotics.CANAnalog;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANAnalog.AnalogMode;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.spartronics4915.lib.hardware.CANCounter;
import com.spartronics4915.lib.util.Logger;

import edu.wpi.first.wpilibj.RobotBase;

public class SpartronicsMax implements SpartronicsMotor
{

    private static final int kVelocitySlotIdx = 0;
    private static final int kPositionSlotIdx = 1;

    private final double kRPMtoRPS = 1.0 / 60.0;

    private final CANSparkMax mSparkMax;
    private final CANSparkMax mFollower;
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

    private CANEncoder mEncoderSensor;
    private CANAnalog mAnalogSensor;
    private CANPIDController mPIDController;
    private AnalogMode mAnalogMode;
    private FeedbackSensorType mFeedbackSensor;

    public class InternalEncoder implements SpartronicsEncoder
    {

        @Override
        public double getVelocity()
        {
            return mSensorModel.toCustomUnits(mEncoderSensor.getVelocity());
        }

        @Override
        public double getPosition()
        {
            return mSensorModel.toCustomUnits(mEncoderSensor.getPosition());
        }

        @Override
        public void setPhase(boolean isReversed)
        {
            mSparkMax.setInverted(isReversed);
        }

        @Override
        public boolean setPosition(double position)
        {
            mEncoderSensor.setPosition(position);
            return true;
        }
    }

    public class AnalogEncoder implements SpartronicsEncoder
    {

        @Override
        public double getVelocity()
        {
            return mSensorModel.toCustomUnits(mAnalogSensor.getVelocity());
        }

        @Override
        public double getPosition()
        {
            return mSensorModel.toCustomUnits(mAnalogSensor.getPosition());
        }

        @Override
        public void setPhase(boolean isReversed)
        {
            mSparkMax.setInverted(isReversed);
        }

        @Override
        public boolean setPosition(double position)
        {
            return false;
        }
    }

    public static enum FeedbackSensorType
    {
        kInternal, kAnalogRelative, kAnalogAbsolute
    }

    // Temporary!!!!
    public static SpartronicsMotor makeMotorBrushed(int deviceNumber)
    {
        if (RobotBase.isSimulation())
        {
            return new SpartronicsSimulatedMotor(deviceNumber);
        }

        Logger.warning("You're using a **temporary** Spark Max brushed constructor! Revert to brushless when you plug in a brushless motor, or be smitten by the Rev Robotics (tm) gods!!!1!");

        return new SpartronicsMax(new CANSparkMax(deviceNumber, MotorType.kBrushed), SensorModel.fromMultiplier(1), FeedbackSensorType.kInternal, null);
    }

    public static SpartronicsMotor makeMotor(int deviceNumber, SensorModel sensorModel,
        FeedbackSensorType feedbackSensor)
    {
        if (RobotBase.isSimulation())
        {
            return new SpartronicsSimulatedMotor(deviceNumber);
        }
        return new SpartronicsMax(new CANSparkMax(deviceNumber, MotorType.kBrushless), sensorModel,
            feedbackSensor, null);
    }

    public static SpartronicsMotor makeMotor(int deviceNumber, SensorModel sensorModel)
    {
        return makeMotor(deviceNumber, sensorModel, FeedbackSensorType.kInternal);
    }

    public static SpartronicsMotor makeMotor(int deviceNumber)
    {
        return makeMotor(deviceNumber, SensorModel.fromMultiplier(1));
    }

    public static SpartronicsMotor makeMotor(int deviceNumber, SensorModel sensorModel,
        FeedbackSensorType feedbackSensor, int followerDeviceNumber)
    {
        if (RobotBase.isSimulation())
        {
            return new SpartronicsSimulatedMotor(deviceNumber, followerDeviceNumber);
        }

        // We only use SPARK MAXes for brushless motors
        // If that changes we can make motor type configurable
        var master = new CANSparkMax(deviceNumber, MotorType.kBrushless);
        CANSparkMax follower = new CANSparkMax(followerDeviceNumber, MotorType.kBrushless);
        follower.follow(master);
        return new SpartronicsMax(master, sensorModel, feedbackSensor, follower);
    }

    public static SpartronicsMotor makeMotor(int deviceNumber, SensorModel sensorModel,
        int followerDeviceNumber)
    {
        return makeMotor(deviceNumber, sensorModel, FeedbackSensorType.kInternal, followerDeviceNumber);
    }

    private SpartronicsMax(CANSparkMax spark, SensorModel sensorModel,
        FeedbackSensorType feedbackSensor, CANSparkMax follower)
    {
        mSparkMax = spark;
        mFollower = follower;
        mSensorModel = sensorModel;
        mPIDController = mSparkMax.getPIDController();
        mFeedbackSensor = feedbackSensor;

        CANError err;
        switch (feedbackSensor)
        {
            case kInternal:
                mEncoderSensor = mSparkMax.getEncoder();
                err = mEncoderSensor.setVelocityConversionFactor(kRPMtoRPS); // Set conversion
                                                                             // factor.
                mEncoder = new InternalEncoder();
                mPIDController.setFeedbackDevice(mEncoderSensor);
                break;
            case kAnalogRelative:
                mAnalogMode = AnalogMode.kRelative;
                mAnalogSensor = mSparkMax.getAnalog(mAnalogMode);
                err = mAnalogSensor.setVelocityConversionFactor(kRPMtoRPS);
                mEncoder = new AnalogEncoder();
                mPIDController.setFeedbackDevice(mAnalogSensor);
                break;
            case kAnalogAbsolute:
                mAnalogMode = AnalogMode.kAbsolute;
                mAnalogSensor = mSparkMax.getAnalog(mAnalogMode);
                err = mAnalogSensor.setVelocityConversionFactor(kRPMtoRPS);
                mEncoder = new AnalogEncoder();
                mPIDController.setFeedbackDevice(mAnalogSensor);
                break;
            default:
                mEncoder = new InternalEncoder();
                err = CANError.kError; // stops errors. Should never happen
        }
        if (err != CANError.kOk)
        {
            Logger.error("SparkMax on with ID " + mSparkMax.getDeviceId()
                + " returned a non-OK error code on sensor configuration... Is the motor controller plugged in?");
            mHadStartupError = true;
        }
        else
        {
            mHadStartupError = false;
        }
        CANCounter.addDevice(mHadStartupError);

        mSparkMax.enableVoltageCompensation(mVoltageCompSaturation);
    }

    @Override
    public SpartronicsEncoder getEncoder()
    {
        return mEncoder;
    }

    /**
     * @return An object for interfacing with a connected analog sensor.
     */
    public CANAnalog getAnalog()
    {
        return mSparkMax.getAnalog(AnalogMode.kAbsolute);
    }

    @Override
    public boolean hadStartupError()
    {
        return false;//mHadStartupError; Change back when comp season is over!
    }

    @Override
    public double getVoltageOutput()
    {
        return mSparkMax.getBusVoltage() * mSparkMax.getAppliedOutput();
    }

    @Override
    public boolean getOutputInverted()
    {
        return mSparkMax.getInverted();
    }

    @Override
    public void setOutputInverted(boolean inverted)
    {
        mSparkMax.setInverted(inverted);
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
        mSparkMax.setIdleMode(mode ? IdleMode.kBrake : IdleMode.kCoast);
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
        mSparkMax.enableVoltageCompensation(mVoltageCompSaturation);
    }

    @Override
    public double getMotionProfileCruiseVelocity()
    {
        return mSensorModel.toCustomUnits(mMotionProfileCruiseVelocity);
    }

    @Override
    public void setMotionProfileCruiseVelocity(double velocityMetersPerSecond)
    { // Set to slot
        mMotionProfileCruiseVelocity = mSensorModel.toNativeUnits(velocityMetersPerSecond);
        mPIDController.setSmartMotionMaxVelocity(mMotionProfileCruiseVelocity,
            kPositionSlotIdx);
    }

    @Override
    public double getMotionProfileMaxAcceleration()
    {
        return mSensorModel.toCustomUnits(mMotionProfileAcceleration);
    }

    @Override
    public void setMotionProfileMaxAcceleration(double accelerationMetersPerSecondSq)
    {
        mMotionProfileAcceleration = mSensorModel.toNativeUnits(accelerationMetersPerSecondSq);
        mPIDController.setSmartMotionMaxAccel(mMotionProfileAcceleration, kPositionSlotIdx);
    }

    @Override
    public void setUseMotionProfileForPosition(boolean useMotionProfile)
    {
        mUseMotionProfileForPosition = useMotionProfile;
        mPIDController.setSmartMotionAllowedClosedLoopError(2.0/360.0, kPositionSlotIdx);
    }

    @Override
    public void setPercentOutput(double dutyCycle, double arbitraryFeedForwardVolts)
    {
        mPIDController.setReference(dutyCycle, ControlType.kDutyCycle, 0,
            arbitraryFeedForwardVolts);
    }

    @Override
    public void setPercentOutput(double dutyCycle)
    {
        setPercentOutput(dutyCycle, 0.0);
    }

    @Override
    public void setVelocity(double velocityMetersPerSecond, double arbitraryFeedForwardVolts)
    {
        double velocityNative = mSensorModel.toNativeUnits(velocityMetersPerSecond);
        mPIDController.setReference(velocityNative, ControlType.kVelocity, kVelocitySlotIdx,
            arbitraryFeedForwardVolts);
    }

    @Override
    public void setVelocityGains(double kP, double kD)
    {
        setVelocityGains(kP, 0, kD, 0);
    }

    @Override
    public void setVelocityGains(double kP, double kI, double kD, double kF)
    {
        mPIDController.setP(kP, kVelocitySlotIdx);
        mPIDController.setI(kI, kVelocitySlotIdx);
        mPIDController.setD(kD, kVelocitySlotIdx);
        mPIDController.setFF(kF, kVelocitySlotIdx);
    }

    @Override
    public void setPosition(double positionMeters)
    {
        double positionNativeUnits = mSensorModel.toNativeUnits(positionMeters);
        mPIDController.setReference(positionNativeUnits,
            mUseMotionProfileForPosition ? ControlType.kSmartMotion : ControlType.kPosition,
            kPositionSlotIdx);
    }

    @Override
    public void setPositionGains(double kP, double kD)
    {
        setPositionGains(kP, 0, kD, 0);
    }

    @Override
    public void setPositionGains(double kP, double kI, double kD, double kF)
    {
        mPIDController.setP(kP, kPositionSlotIdx);
        mPIDController.setI(kI, kPositionSlotIdx);
        mPIDController.setD(kD, kPositionSlotIdx);
        mPIDController.setFF(kF, kPositionSlotIdx);
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
        mPIDController.setReference(0.0, ControlType.kDutyCycle, 0);
    }

    public FeedbackSensorType getFeedbackSensor()
    {
        return mFeedbackSensor;
    }

    @Override
    public double getOutputCurrent()
    {
        return mSparkMax.getOutputCurrent();
    }

    @Override
    public SpartronicsMotor getFollower()
    {
        return new SpartronicsMax(mFollower, this.mSensorModel, this.mFeedbackSensor, null);
    }

    @Override
    public int getDeviceNumber()
    {
        return mSparkMax.getDeviceId();
    }

    @Override
    public void setSoftLimits(double forwardLimitCustomUnits, double reverseLimitCustomUnits)
    {
        mSparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
        mSparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true);

        mSparkMax.setSoftLimit(SoftLimitDirection.kForward, (float) mSensorModel.toNativeUnits(forwardLimitCustomUnits));
        mSparkMax.setSoftLimit(SoftLimitDirection.kReverse, (float) mSensorModel.toNativeUnits(reverseLimitCustomUnits));
    }

    @Override
    public void setStatorCurrentLimit(int limitAmps)
    {
        mSparkMax.setSmartCurrentLimit(limitAmps);
    }

}
