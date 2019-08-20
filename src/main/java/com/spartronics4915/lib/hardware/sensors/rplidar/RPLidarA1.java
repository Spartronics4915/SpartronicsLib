package com.spartronics4915.lib.hardware.sensors.rplidar;

import java.util.function.Consumer;

public class RPLidarA1 extends RPLidarDevice {

    public RPLidarA1(String usbPortPath, Consumer<RPLidarMeasurement> measurementConsumer) {
        super(usbPortPath, measurementConsumer);
    }

    public RPLidarA1(String usbPortPath, PointcloudConsumer pointcloudConsumer) {
        super(usbPortPath, pointcloudConsumer);
    }

	@Override
    public void stop() {
        mIsClosing.set(true);
        mDriver.sendStop(false);
    }

}