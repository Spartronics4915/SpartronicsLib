package  com.spartronics4915.lib.hardware.sensors.rplidar;

/**
 * Listener for client of {@link RPLidarLowLevelDriver}
 *
 * @author Peter Abeles
 */
public interface RPLidarListener {

	void handleMeasurement(RPLidarMeasurement measurement);

	void handleDeviceHealth(RPLidarHealth health);

	void handleDeviceInfo(RPLidarDeviceInfo info);
}
