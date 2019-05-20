package com.spartronics4915.lib.sensors;

import edu.wpi.first.wpilibj.AnalogInput;

public abstract class IRSensor
{
    AnalogInput mAnalogInput;

    public IRSensor(int port)
    {
        mAnalogInput = new AnalogInput(port);
        mAnalogInput.setAverageBits(12);
    }

    public double getVoltage()
    {
        double v = mAnalogInput.getAverageVoltage();
        if (v < .001)
            v = .001;
        return v;
    }

    public abstract double getDistance(); // inches

    public boolean isTargetInVoltageRange(double min, double max)
    {
        double v = getVoltage();
        return v > min && v < max;
    }

    /**
     * @param minDist in inches
     * @param maxDist in inches
     * @return is within the distance
     */
    public boolean isTargetInDistanceRange(double minDist, double maxDist)
    {
        double d = getDistance();
        return d > minDist && d < maxDist;
    }
}