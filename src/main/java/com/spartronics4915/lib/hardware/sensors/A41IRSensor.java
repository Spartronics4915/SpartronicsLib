package com.spartronics4915.lib.hardware.sensors;

public class A41IRSensor extends IRSensor
{

    public A41IRSensor(int port)
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
       // double cm = 59.06 - (94.11 * volt) + (57.60 * Math.pow(volt, 2.0)) - (11.65 * Math.pow(volt, 3.0));
        double cm = 50.9443 - (65.28722 * volt) + (32.55266 * Math.pow(volt, 2.0)) - (5.497291 * Math.pow(volt, 3.0));
       return cm;
    }
}
