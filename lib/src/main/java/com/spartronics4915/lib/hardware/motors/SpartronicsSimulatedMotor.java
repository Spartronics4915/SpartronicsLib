package com.spartronics4915.lib.hardware.motors;

import java.util.HashMap;

/**
 * This class provides a simulated, easy-to-inspect, implementor of SpartronicsMotor.
 */
// TODO: Keep track of motor state
public class SpartronicsSimulatedMotor implements SpartronicsMotor
{
    private static HashMap<Integer, SpartronicsSimulatedMotor> sMotors = new HashMap<>();

    public static SpartronicsSimulatedMotor getFromId(int deviceId)
    {
        var dev = sMotors.get(deviceId);
        if (dev == null)
        {
            throw new RuntimeException("Simulated motor with ID " + deviceId + " not found");
        }
        return dev;
    }

    public static void resetGlobalState()
    {
        sMotors = new HashMap<>();
    }

    private double mMotionProfileMaxAcceleration = 0;
    private boolean mOutputInverted = false;
    private boolean mBrakeMode = false;
    private double mVoltageCompSaturation = 0;
    private double mMotionProfileCruiseVelocity = 0;
    private double mOutputCurrent = 0;

    private final SpartronicsSimulatedMotor mFollower;
    private final int mDeviceNumber;

    @Override
    public SpartronicsEncoder getEncoder()
    {
        return new SpartronicsEncoder()
        {
            private double mPosition = 0;

            @Override
            public void setPhase(boolean isReversed)
            {
            }

            @Override
            public double getVelocity()
            {
                return 0;
            }

            @Override
            public double getPosition()
            {
                return mPosition;
            }

            @Override
            public boolean setPosition(double position)
            {
                mPosition = position;
                return true;
            }
        };
    }

    public SpartronicsSimulatedMotor(int deviceNumber)
    {
        this(deviceNumber, -1);
    }

    public SpartronicsSimulatedMotor(int deviceNumber, int followerDeviceNumber)
    {
        if (sMotors.get(deviceNumber) != null)
        {
            throw new RuntimeException(
                "Can't instantiate a duplicate motor with device ID " + deviceNumber);
        }
        sMotors.put(deviceNumber, this);

        mDeviceNumber = deviceNumber;

        if (followerDeviceNumber != -1)
        {
            mFollower = new SpartronicsSimulatedMotor(followerDeviceNumber);
        }
        else
        {
            mFollower = null;
        }
    }

    @Override
    public SensorModel getSensorModel()
    {
        return SensorModel.fromMultiplier(1);
    }

    @Override
    public boolean hadStartupError()
    {
        return false;
    }

    @Override
    public double getVoltageOutput()
    {
        return 0;
    }

    @Override
    public boolean getOutputInverted()
    {
        return mOutputInverted;
    }

    @Override
    public void setOutputInverted(boolean inverted)
    {
        mOutputInverted = inverted;
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
    }

    @Override
    public double getMotionProfileCruiseVelocity()
    {
        return mMotionProfileCruiseVelocity;
    }

    @Override
    public void setMotionProfileCruiseVelocity(double velocityMetersPerSecond)
    {
        mMotionProfileCruiseVelocity = velocityMetersPerSecond;
    }

    @Override
    public double getMotionProfileMaxAcceleration()
    {
        return mMotionProfileMaxAcceleration;
    }

    @Override
    public void setMotionProfileMaxAcceleration(double accelerationMetersSecSq)
    {
        mMotionProfileMaxAcceleration = accelerationMetersSecSq;
    }

    @Override
    public void setUseMotionProfileForPosition(boolean useMotionProfile)
    {

    }

    @Override
    public void setPercentOutput(double dutyCycle, double arbitraryFeedForwardVolts)
    {

    }

    @Override
    public void setPercentOutput(double dutyCycle)
    {

    }

    @Override
    public void setVelocity(double velocityMetersPerSecond)
    {

    }

    @Override
    public void setVelocity(double velocityMetersPerSecond, double arbitraryFeedForwardVolts)
    {

    }

    @Override
    public void setVelocityGains(double kP, double kD)
    {

    }

    @Override
    public void setVelocityGains(double kP, double kI, double kD, double kF)
    {

    }

    @Override
    public void setPosition(double positionMeters)
    {

    }

    @Override
    public void setPositionGains(double kP, double kD)
    {

    }

    @Override
    public void setPositionGains(double kP, double kI, double kD, double kF)
    {

    }

    @Override
    public void setNeutral()
    {

    }

    @Override
    public double getOutputCurrent()
    {
        return mOutputCurrent;
    }

    public void setOutputCurrent(double current)
    {
        mOutputCurrent = current;
    }

    @Override
    public SpartronicsMotor getFollower()
    {
        return mFollower;
    }

    @Override
    public int getDeviceNumber()
    {
        return mDeviceNumber;
    }

    @Override
    public void setSoftLimits(double forwardLimitCustomUnits, double reverseLimitCustomUnits)
    {

    }

}
