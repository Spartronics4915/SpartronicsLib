package com.spartronics4915.lib.math.twodim.physics;

import com.spartronics4915.lib.math.Util;

/**
 * Model of a DC motor rotating a shaft. All parameters refer to the output
 * (e.g. should already consider gearing
 * and efficiency losses). The motor is assumed to be symmetric forward/reverse.
 */
public class DCMotorTransmission
{
    // All units must be SI!
    protected final double mSpeedPerVolt; // rad/s per V (no load)
    protected final double mTorquePerVolt; // N m per V (stall)
    protected final double mFrictionVoltage; // V

    public DCMotorTransmission(final double speedPerVolt,
            final double torquePerVolt,
            final double frictionVoltage)
    {
        mSpeedPerVolt = speedPerVolt;
        mTorquePerVolt = torquePerVolt;
        mFrictionVoltage = frictionVoltage;
    }

    public double speedPerVolt()
    {
        return mSpeedPerVolt;
    }

    public double torquePerVolt()
    {
        return mTorquePerVolt;
    }

    public double frictionVoltage()
    {
        return mFrictionVoltage;
    }

    public double freeSpeedAtVoltage(final double voltage)
    {
        if (voltage > Util.kEpsilon)
        {
            return Math.max(0.0, voltage - frictionVoltage()) * speedPerVolt();
        }
        else if (voltage < -Util.kEpsilon)
        {
            return Math.min(0.0, voltage + frictionVoltage()) * speedPerVolt();
        }
        else
        {
            return 0.0;
        }
    }

    public double getTorqueForVoltage(final double outputSpeed, final double voltage)
    {
        double effectiveVoltage = voltage;
        if (outputSpeed > Util.kEpsilon)
        {
            // Forward motion, rolling friction.
            effectiveVoltage -= frictionVoltage();
        }
        else if (outputSpeed < -Util.kEpsilon)
        {
            // Reverse motion, rolling friction.
            effectiveVoltage += frictionVoltage();
        }
        else if (voltage > Util.kEpsilon)
        {
            // System is static, forward torque.
            effectiveVoltage = Math.max(0.0, voltage - frictionVoltage());
        }
        else if (voltage < -Util.kEpsilon)
        {
            // System is static, reverse torque.
            effectiveVoltage = Math.min(0.0, voltage + frictionVoltage());
        }
        else
        {
            // System is idle.
            return 0.0;
        }
        return torquePerVolt() * (-outputSpeed / speedPerVolt() + effectiveVoltage);
    }

    public double getVoltageForTorque(final double outputSpeed, final double torque)
    {
        double frictionVoltage;
        if (outputSpeed > Util.kEpsilon)
        {
            // Forward motion, rolling friction.
            frictionVoltage = frictionVoltage();
        }
        else if (outputSpeed < -Util.kEpsilon)
        {
            // Reverse motion, rolling friction.
            frictionVoltage = -frictionVoltage();
        }
        else if (torque > Util.kEpsilon)
        {
            // System is static, forward torque.
            frictionVoltage = frictionVoltage();
        }
        else if (torque < -Util.kEpsilon)
        {
            // System is static, reverse torque.
            frictionVoltage = -frictionVoltage();
        }
        else
        {
            // System is idle.
            return 0.0;
        }
        return torque / torquePerVolt() + outputSpeed / speedPerVolt() + frictionVoltage;
    }
}
