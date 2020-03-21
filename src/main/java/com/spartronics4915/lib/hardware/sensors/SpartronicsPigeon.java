package com.spartronics4915.lib.hardware.sensors;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.spartronics4915.lib.hardware.CANCounter;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class SpartronicsPigeon implements SpartronicsIMU
{
    PigeonIMU mIMU;

    public SpartronicsPigeon(int canID)
    {
        PigeonIMU imu = new PigeonIMU(canID);
        ErrorCode err = imu.configFactoryDefault();

        boolean hadError = err != ErrorCode.OK;
        CANCounter.addDevice(hadError);

        mIMU = imu;
    }

    @Override
    public Rotation2d getYaw()
    {
        return Rotation2d.fromDegrees(mIMU.getFusedHeading());
    }
}
