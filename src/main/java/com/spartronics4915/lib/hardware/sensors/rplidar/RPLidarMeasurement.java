package  com.spartronics4915.lib.hardware.sensors.rplidar;

/**
 * Single measurement from LIDAR
 *
 * @author Peter Abeles
 */
public class RPLidarMeasurement {

	/** Is this the start of a new scan */
	public boolean start;
	/** Scan quality from 0 (lowest) to 255 (highest) */
	public int quality;
	/** Angle in degrees */
	public float angle;
	/** Distance in millimeters */
	public float distance;
	/** System.currentTimeMillis() when the measurement arrived */
	public long timestamp;

	public boolean isInvalid() {
		return distance == 0;
	}

	public String toString() {
		return "Starting: " + start + ", Quality: " + quality + ", Angle: " + angle + ", Distance: " + distance
				+ ", Timestamp: " + timestamp;
	}
}
