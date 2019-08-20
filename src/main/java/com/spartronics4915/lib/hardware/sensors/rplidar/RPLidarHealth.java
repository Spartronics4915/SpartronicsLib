package  com.spartronics4915.lib.hardware.sensors.rplidar;

/**
 * Packet which describes the sensor's health
 *
 * @author Peter Abeles
 * @author Juan Antonio Breña Moral
 */
public class RPLidarHealth {

	public int status;
	public int errorCode;

	public void print() {
		System.out.println("HEALTH:");
		switch (status) {
		case 0:
			System.out.println("  Good");
			break;
		case 1:
			System.out.println("  Warning");
			break;
		case 2:
			System.out.println("  Error");
			break;
		default:
			System.out.println("  unknown = " + status);
		}
		System.out.println("  error_code = " + errorCode);
	}
}
