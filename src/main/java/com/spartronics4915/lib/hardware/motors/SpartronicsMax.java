/************************************************\
| Before we begin, here's a word from our        |
| sponsor, RAID: Shadow Legends.                 |
| RAID: Shadow Legends™ is                       |
| an immersive online experience with everything |
| you'd expect from a brand new RPG title. It's  |
| got an amazing storyline, awesome 3D graphics, |
| giant boss fights, PVP battles, and hundreds   |
| of never before seen champions to collect and  |
| customize.                                     |
| I never expected to get this level of          |
| performance out of a mobile game. Look how     |
| crazy the level of detail is on these          |
| champions!                                     |
| RAID: Shadow Legends™ is getting big real      |
| fast, so you should definitely get in early.   |
| Starting now will give you a huge head start.  |
| There's also an upcoming Special Launch        |
| Tournament with crazy prizes! And not to       |
| mention, this game is absolutely free!         |
| So go ahead and check out the video            |
| description to find out more about             |
| RAID: Shadow Legends™. There, you will find a  |
| link to the store page and a special code to   |
| unlock all sorts of goodies. Using the special |
| code, you can get 50,000 Silver immediately,   |
| and a FREE Epic Level Champion as part of the  |
| new players program, courtesy of course of the |
| RAID: Shadow Legends™ devs.                    |
| DISCLAIMER: Not actually sponsored by RAID:    |
| Shadow Legends                                 |
\************************************************/


package com.spartronics4915.lib.hardware.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SpartronicsMax implements SpartronicsMotor {

    private static final int kVelocitySlotIdx = 0;
    private static final int kPositionSlotIdx = 1;

    private final double kRPMtoRPS = 1/60;

    private CANSparkMax mSparkMax;
    private SpartronicsEncoder mEncoder;
    private SensorModel mSensorModel;

    private boolean mBrakeMode = false;
    /** Volts */
    private double mVoltageCompSaturation = 12.0;
    /** Native units/sec, converted to meters on get and set */
    private double mMotionProfileCruiseVelocity = 0.0;
    /** Native units/sec^2, converted to meters on get and set */
    private double mMotionProfileAcceleration = 0.0;
    private boolean mUseMotionProfileForPosition = false;

    public class SpartronicsMaxEncoder implements SpartronicsEncoder {

        @Override
        public double getVelocity() {
            return mSensorModel.toCustomUnits(mSparkMax.getEncoder().getVelocity());
        }

        @Override
        public double getPosition() {
            return mSensorModel.toCustomUnits(mSparkMax.getEncoder().getPosition());
        }

        @Override
        public void setPhase(boolean isReversed) {
            mSparkMax.setInverted(isReversed);
        }
    }

    public SpartronicsMax(int deviceNumber, SensorModel sensorModel) {
        this(new CANSparkMax(deviceNumber, MotorType.kBrushless), sensorModel);
    }

    public SpartronicsMax(CANSparkMax spark, SensorModel sensorModel) {
        mSparkMax = spark;
        mSensorModel = sensorModel;
        mEncoder = new SpartronicsMaxEncoder();
        mSparkMax.getEncoder().setPosition(0);
        mSparkMax.getEncoder().setVelocityConversionFactor(kRPMtoRPS); // Set conversion factor.

        // mSparkMax.configFactoryDefault();
        mSparkMax.enableVoltageCompensation(mVoltageCompSaturation);
    }

    @Override
    public SpartronicsEncoder getEncoder() {
        return mEncoder;
    }

    @Override
    public double getVoltageOutput() {
        return mSparkMax.getBusVoltage() * mSparkMax.getAppliedOutput();
    }

    @Override
    public boolean getOutputInverted() {
        return mSparkMax.getInverted();
    }

    @Override
    public void setOutputInverted(boolean inverted) {
        mSparkMax.setInverted(inverted);
    }

    @Override
    public boolean getBrakeMode() {
        return mBrakeMode;
    }

    @Override
    public void setBrakeMode(boolean mode) {
        mBrakeMode = mode;
        mSparkMax.setIdleMode(mode ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public double getVoltageCompSaturation() {
        return mVoltageCompSaturation;
    }

    @Override
    public void setVoltageCompSaturation(double voltage) {
        mVoltageCompSaturation = voltage;
        mSparkMax.enableVoltageCompensation(mVoltageCompSaturation);
    }

    @Override
    public double getMotionProfileCruiseVelocity() {
        return mSensorModel.toCustomUnits(mMotionProfileCruiseVelocity);
    }

    @Override
    public void setMotionProfileCruiseVelocity(double velocityMetersPerSecond) { // Set to slot
        mMotionProfileCruiseVelocity = mSensorModel
                .toNativeUnits(velocityMetersPerSecond);
        mSparkMax.getPIDController().setSmartMotionMaxVelocity((int) mMotionProfileCruiseVelocity, kVelocitySlotIdx);
    }

    @Override
    public double getMotionProfileMaxAcceleration() {
        return mSensorModel.toCustomUnits(mMotionProfileAcceleration);
    }

    @Override
    public void setMotionProfileMaxAcceleration(double accelerationMetersPerSecondSq) {
        mMotionProfileAcceleration = mSensorModel
                .toNativeUnits(accelerationMetersPerSecondSq);
        mSparkMax.getPIDController().setSmartMotionMaxAccel((int) mMotionProfileAcceleration, kVelocitySlotIdx);
    }

    @Override
    public void setUseMotionProfileForPosition(boolean useMotionProfile) {
        mUseMotionProfileForPosition = useMotionProfile;
    }

    @Override
    public void setDutyCycle(double dutyCycle, double arbitraryFeedForwardVolts) {
        mSparkMax.getPIDController().setReference(dutyCycle, ControlType.kDutyCycle, 0, 
        arbitraryFeedForwardVolts);
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        setDutyCycle(dutyCycle, 0.0);
    }

    @Override
    public void setVelocity(double velocityMetersPerSecond, double arbitraryFeedForwardVolts) {
        double velocityNative = mSensorModel.toNativeUnits(velocityMetersPerSecond);
        mSparkMax.getPIDController().setReference(velocityNative, ControlType.kVelocity, kVelocitySlotIdx,
        arbitraryFeedForwardVolts);
    }

    @Override
    public void setVelocityGains(double kP, double kD) {
        setVelocityGains(kP, 0, kD, 0);
    }

    @Override
    public void setVelocityGains(double kP, double kI, double kD, double kF) {
        mSparkMax.getPIDController().setP(kP, kVelocitySlotIdx);
        mSparkMax.getPIDController().setI(kI, kVelocitySlotIdx);
        mSparkMax.getPIDController().setD(kD, kVelocitySlotIdx);
        mSparkMax.getPIDController().setFF(kF, kVelocitySlotIdx);
    }

    @Override
    public void setPosition(double positionMeters) {
        positionMeters = mSensorModel.toNativeUnits(positionMeters);
        mSparkMax.getPIDController().setReference(positionMeters, mUseMotionProfileForPosition ? ControlType.kSmartMotion : ControlType.kPosition, kPositionSlotIdx);
    }

    @Override
    public void setPositionGains(double kP, double kD) {
        setPositionGains(kP, 0, kD, 0);
    }

    @Override
    public void setPositionGains(double kP, double kI, double kD, double kF) {
        mSparkMax.getPIDController().setP(kP, kPositionSlotIdx);
        mSparkMax.getPIDController().setI(kI, kPositionSlotIdx);
        mSparkMax.getPIDController().setD(kD, kPositionSlotIdx);
        mSparkMax.getPIDController().setFF(kF, kPositionSlotIdx);
    }

    public void follow(SpartronicsMax other) {
        mSparkMax.follow(other.mSparkMax);
    }

    @Override
    public SensorModel getSensorModel() {
        return mSensorModel;
    }

    @Override
    public void setVelocity(double velocityMetersPerSecond) {
        setVelocity(velocityMetersPerSecond, 0.0);
    }

    @Override
    public void setNeutral() {
        mSparkMax.getPIDController().setReference(0.0, ControlType.kDutyCycle, 0);
    }

}
