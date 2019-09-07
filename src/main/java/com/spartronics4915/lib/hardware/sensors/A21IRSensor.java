package com.spartronics4915.lib.hardware.sensors;

public class A21IRSensor extends IRSensor
{

    public A21IRSensor(int port)
    {
        super(port);
    }

    //We got this by the same method as used in the A51 sensor, using the sensor to measure 9 different cm values
    //and put it through a cubic regression.

    @Override
    public double getDistance()
    {
        double volt = getVoltage();
        double cm = 356.3748 - (1576.436 * volt) + (2461.958 * Math.pow(volt, 2.0)) - (1291.155 * Math.pow(volt, 3.0));
        return cm / 2.54;
    }

}
