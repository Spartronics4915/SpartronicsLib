package com.spartronics4915.lib.hardware.sensors;

public class A02IRSensor extends IRSensor
{

    public A02IRSensor(int port)
    {
        super(port);
    }

    @Override
    public double getDistance()
    {
        // formula for sharp a41 detector, each model has a different formula
        //      v = 1 / (L + .42)  (1/cm)
        //
        //double cm = 1.0 / getVoltage() - .42; // warning blows up when v == 0
        double volt = getVoltage();
        double cm = 56.2958 - (79.12745 * volt) + (43.04363 * Math.pow(volt, 2.0)) - (7.753581 * Math.pow(volt, 3.0));
       return cm / 2.54;
    }
}
