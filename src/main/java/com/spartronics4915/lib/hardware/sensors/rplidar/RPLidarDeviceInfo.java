package  com.spartronics4915.lib.hardware.sensors.rplidar;

/**
 * Contains information about the device
 *
 * @author Peter Abeles
 * @author Juan Antonio Breña Moral
 */
public class RPLidarDeviceInfo {

	public int model;
	public int firmwareMinor;
	public int firmwareMajor;
	public int hardware;
	public byte[] serialNumber = new byte[16];

	public void print() {

		System.out.println("DEVICE INFO");
		System.out.println("  model = " + model);
		System.out.println("  firmware_minor = " + firmwareMinor);
		System.out.println("  firmware_major = " + firmwareMajor);
		System.out.println("  hardware = " + hardware);

		final StringBuilder sb = new StringBuilder();
		sb.append("  Serial = ");
		for (int i = 0; i < serialNumber.length; i++) {
			sb.append(String.format("%02X", serialNumber[i]));
			if ((i + 1) % 4 == 0)
				sb.append(" ");
		}
		System.out.println(sb.toString());
	}
}
