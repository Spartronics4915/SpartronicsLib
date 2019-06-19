package com.spartronics4915.lib.hardware.sensors;

public class A51IRSensor extends IRSensor
{

    public A51IRSensor(int port)
    {
        super(port);
    }

    //How did we get here? We measured 9 seperate centimeter values and got the voltage. 
    //We then used a cubic polynomial regression to find the equation below.
    //Range: 10 cm - 80 cm

    @Override
    public double getDistance()
    {
        double volt = getVoltage();
        double cm = 56.2958 - (79.12745 * volt) + (43.04363 * Math.pow(volt, 2.0)) - (7.753581 * Math.pow(volt, 3.0));
        return cm / 2.54;
    }
}
