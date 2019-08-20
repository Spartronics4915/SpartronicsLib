package com.spartronics4915.lib.hardware.sensors.rplidar;

import java.util.function.Consumer;

/**
 * Class to control all RPLidar A2 devices. Note that the lower level device
 * driver does not currently support scanning in express mode, so you will not
 */
public class RPLidarA2 extends RPLidarDevice {

    public RPLidarA2(String usbPortPath, Consumer<RPLidarMeasurement> measurementConsumer) {
        super(usbPortPath, measurementConsumer);
    }

    public RPLidarA2(String usbPortPath, PointcloudConsumer pointcloudConsumer) {
        super(usbPortPath, pointcloudConsumer);
    }

    @Override
    public void stop() {
        mIsClosing.set(true);
        mDriver.sendStop(true);
    }

    /**
     * This sets the PWM duty cycle of the motor. This operation is not supported on
     * all devices, and is not directly related to any physical angular velocity.
     * 
     * @param motorPWM Motor PWM duty cycle on the range of [0, 1023].
     */
    public void setMotorPWM(int motorPWM) {
        if (motorPWM < 0 || motorPWM > 1023) {
            throw new RPLidarException("motorPWM must be on the range of [0, 1023]");
        }

        mMotorPWM = motorPWM;
        mDriver.sendStartMotor(mMotorPWM);
    }
}