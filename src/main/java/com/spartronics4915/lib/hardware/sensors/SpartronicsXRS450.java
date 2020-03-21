package com.spartronics4915.lib.hardware.sensors;

import edu.wpi.first.wpilibj.geometry.Rotation2d;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;

public class SpartronicsXRS450 implements SpartronicsIMU
{
    private ADXRS450_Gyro mGyro = new ADXRS450_Gyro();

    @Override
    public Rotation2d getYaw()
    {
        return Rotation2d.fromDegrees(-mGyro.getAngle());
    }
}
