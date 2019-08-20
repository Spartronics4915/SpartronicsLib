package com.spartronics4915.lib.hardware.sensors.rplidar;

import purejavacomm.CommPort;
import purejavacomm.CommPortIdentifier;
import purejavacomm.SerialPort;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Low level service for RPLidar. Just sends and receives packets. Doesn't
 * attempt to filter bad data or care about timeouts.
 *
 * @author Peter Abeles, Declan Freeman-Gleason
 */
public class RPLidarLowLevelDriver {

	// out going packet types
	private static final byte SYNC_BYTE0 = (byte) 0xA5;
	private static final byte SYNC_BYTE1 = (byte) 0x5A;
	private static final byte STOP = (byte) 0x25;
	private static final byte RESET = (byte) 0x40;
	private static final byte SCAN = (byte) 0x20;
	private static final byte EXPRESS_SCAN = (byte) 0x82;
	private static final byte FORCE_SCAN = (byte) 0x21;
	private static final byte GET_INFO = (byte) 0x50;
	private static final byte GET_HEALTH = (byte) 0x52;
	private static final byte START_MOTOR = (byte) 0xF0;

	// in coming packet types
	private static final byte RCV_INFO = (byte) 0x04;
	private static final byte RCV_HEALTH = (byte) 0x06;
	private static final byte RCV_SCAN = (byte) 0x81;

	SerialPort mSerialPort;
	InputStream mInStream;
	OutputStream mOutStream;

	// buffer for out going data
	byte[] mDataOutBuf = new byte[1024];

	// flag to turn on and off verbose debugging output
	boolean mVerbose = false;

	// thread for reading serial data
	private ReadSerialThread mReadThread;

	// Storage for incoming packets
	RPLidarHealth mLatestHealth = new RPLidarHealth();
	RPLidarDeviceInfo mLatestDeviceInfo = new RPLidarDeviceInfo();
	RPLidarListener mListener;

	// if it is in scanning mode. When in scanning mode it just parses measurement
	// packets
	boolean scanning = false;

	// Type of packet last recieved
	int lastReceived = 0;

	/**
	 * Initializes serial connection
	 *
	 * @param portName Path to serial port
	 * @param listener Listener for in comming packets
	 * @throws Exception
	 */
	public RPLidarLowLevelDriver(final String portName, final RPLidarListener listener) throws Exception {

		System.out.println("Opening port " + portName);

		this.mListener = listener;

		// Configuration for Serial port operations
		final CommPortIdentifier portIdentifier = CommPortIdentifier.getPortIdentifier(portName);
		final CommPort commPort = portIdentifier.open("RPLidar4J", 2000);
		mSerialPort = (SerialPort) commPort;
		mSerialPort.setSerialPortParams(115200, SerialPort.DATABITS_8, SerialPort.STOPBITS_1, SerialPort.PARITY_NONE);
		mSerialPort.setFlowControlMode(SerialPort.FLOWCONTROL_NONE);

		mInStream = mSerialPort.getInputStream();
		mOutStream = mSerialPort.getOutputStream();

		mReadThread = new ReadSerialThread();
		new Thread(mReadThread).start();
	}

	/**
	 * Pauses for the specified number of milliseconds
	 */
	// TODO Refactor using Delay
	public void pause(long milli) {
		synchronized (this) {
			try {
				wait(milli);
			} catch (InterruptedException e) {
			}
		}
	}

	/**
	 * Shuts down the serial connection and threads
	 */
	public void shutdown() {

		mSerialPort.close();

		// TODO Check by the status of the Thread
		if (mReadThread != null) {
			mReadThread.requestStop();
			mReadThread = null;
		}
	}

	/**
	 * Request that it enter scan mode
	 *
	 * @param timeout Blocking time. Resends packet periodically. <= 0 means no
	 *                blocking.
	 * @return true if successful
	 */
	public boolean sendScan(final long timeout) {
		return sendBlocking(SCAN, RCV_SCAN, timeout);
	}

	/**
	 * Request that the LIDAR enters the fastest possible scan mode.
	 * 
	 * I'm keeping this here for completeness, but it's unused because it doesn't
	 * yield a higher sampling rate for the A1 devices. If you want to use the
	 * higher sampling rate because you have an A2 device you will need to call this
	 * method instead of sendScan, and you will also need to implement parsing of
	 * express scan response packets, which are different than the packets for a
	 * regular scan.
	 * 
	 * @param timeout Blocking time. Resends packet periodically if >= 0.
	 * @return true if successful
	 */
	public boolean sendExpressScan(final long timeout) {
		var expressPayload = new byte[] { (byte) 0x05, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00,
				(byte) 0x22 };

		return sendBlocking(EXPRESS_SCAN, expressPayload, (byte) 0x82, timeout);
	}

	/**
	 * Sends a reset packet which will put it into its initial state
	 */
	public void sendReset() {
		scanning = false;
		sendNoPayLoad(RESET);
	}

	/**
	 * Requests that a sensor info packet be sent
	 *
	 * @param timeout Blocking time. Resends packet periodically. <= 0 means no
	 *                blocking.
	 * @return true if successful
	 */
	public boolean sendGetInfo(final long timeout) {
		return sendBlocking(GET_INFO, RCV_INFO, timeout);
	}

	/**
	 * Requests that a sensor health packet be sent
	 *
	 * @param timeout Blocking time. Resends packet periodically. <= 0 means no
	 *                blocking.
	 * @return true if successful
	 */
	public boolean sendGetHealth(final long timeout) {
		return sendBlocking(GET_HEALTH, RCV_HEALTH, timeout);
	}

	/**
	 * Low level blocking packet send routine
	 */
	protected boolean sendBlocking(byte command, byte expected, long timeout) {
		return sendBlocking(command, null, expected, timeout);
	}

	protected boolean sendBlocking(byte command, byte[] payload, byte expected, long timeout) {
		Runnable sender = (payload == null) ? (() -> sendNoPayLoad(command)) : (() -> sendPayLoad(command, payload));

		if (timeout <= 0) {
			sender.run();
			return true;
		} else {
			lastReceived = 0;
			long endTime = System.currentTimeMillis() + timeout;
			do {
				sender.run();
				pause(20);
			} while (endTime >= System.currentTimeMillis() && lastReceived != expected);
			return lastReceived == expected;
		}
	}

	/**
	 * Sends a command with no data payload
	 */
	protected void sendNoPayLoad(byte command) {
		if (mVerbose) {
			System.out.printf("Sending command 0x%02x\n", command & 0xFF);
		}

		mDataOutBuf[0] = SYNC_BYTE0;
		mDataOutBuf[1] = command;

		try {
			mOutStream.write(mDataOutBuf, 0, 2);
			mOutStream.flush();
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
	}

	/**
	 * Sends a command with data payload
	 */
	protected void sendPayLoad(byte command, byte[] payLoad) {
		if (mVerbose) {
			System.out.printf("Sending command 0x%02x\n", command);
		}

		mDataOutBuf[0] = SYNC_BYTE0;
		mDataOutBuf[1] = command;
		mDataOutBuf[2] = (byte) (payLoad.length & 0xFF);

		// Calculate checksum
		// Docs say it should be
		// 0 ⨁ 0xA5 ⨁ CmdType ⨁ PayloadSize ⨁ Payload[0] ⨁ ... ⨁ Payload[n]
		int checksum = 0 ^ mDataOutBuf[0] ^ mDataOutBuf[1] ^ mDataOutBuf[2];

		for (int i = 0; i < payLoad.length; i++) {
			mDataOutBuf[3 + i] = payLoad[i];
			checksum ^= mDataOutBuf[3 + i];
		}

		// add checksum - now total length is 3 + payLoad.length + 1
		mDataOutBuf[3 + payLoad.length] = (byte) checksum;

		// System.out.print("dataOut = [");
		// for (byte b : dataOut) {
		// System.out.printf("0x%02x ", b);
		// }
		// System.out.println("] (We will send " + (3 + payLoad.length + 1) + ")");

		try {
			mOutStream.write(mDataOutBuf, 0, 3 + payLoad.length + 1);
			mOutStream.flush();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	protected byte[] toLittleEndian(int payLoadInt) {
		byte[] payload = new byte[2];

		// load payload little Endian
		payload[0] = (byte) (payLoadInt & 0xFF);
		payload[1] = (byte) ((payLoadInt >> 8) & 0xFF);

		return payload;
	}

	/**
	 * Sends a start motor command
	 */
	public void sendStartMotor(int speed) {
		mSerialPort.setDTR(false);
		sendPayLoad(START_MOTOR, toLittleEndian(speed));
	}

	/**
	 * Sends a stop and stop motor command.
	 * 
	 * @param isA2 Is this an A2 device. If true we will send a motor speed command
	 *             instead of a STOP command.
	 */
	public void sendStop(boolean isA2) {
		scanning = false;
		if (isA2) {
			sendStartMotor(0);
		} else {
			sendNoPayLoad(STOP);
			pause(200);
			mSerialPort.setDTR(true); // This is actually what makes things stop
		}
	}

	/**
	 * Searches for and parses all complete packets inside data
	 */
	protected int parseData(byte[] data, int length) {

		int offset = 0;

		// search for the first good packet it can find
		while (true) {
			if (scanning) {
				if (offset + 5 > length) {
					return offset;
				}

				if (parseScan(data, offset, 5)) {
					offset += 5;
				} else {
					if (mVerbose)
						System.out.println("--- Bad Packet ---");
					offset += 1;
				}
			} else {
				// see if it has consumed all the data
				if (offset + 1 + 4 + 1 > length) {
					return offset;
				}

				byte start0 = data[offset];
				byte start1 = data[offset + 1];

				if (start0 == SYNC_BYTE0 && start1 == SYNC_BYTE1) {
					int info = ((data[offset + 2] & 0xFF)) | ((data[offset + 3] & 0xFF) << 8)
							| ((data[offset + 4] & 0xFF) << 16) | ((data[offset + 5] & 0xFF) << 24);

					int packetLength = info & 0x3FFFFFFF;
					// int sendMode = (info >> 30) & 0xFF;
					byte dataType = data[offset + 6];

					if (mVerbose) {
						System.out.printf("packet 0x%02x length = %d\n", dataType, packetLength);
					}
					// see if it has received the entire packet
					if (offset + 2 + 5 + packetLength > length) {
						if (mVerbose) {
							System.out.println("  waiting for rest of the packet");
						}
						return offset;
					}

					if (parsePacket(data, offset + 2 + 4 + 1, packetLength, dataType)) {
						lastReceived = dataType & 0xFF;
						offset += 2 + 5 + packetLength;
					} else {
						offset += 2;
					}
				} else {
					offset++;
				}
			}
		}
	}

	protected boolean parsePacket(byte[] data, int offset, int length, byte type) {
		switch (type) {
		case (byte) RCV_INFO: // INFO
			return parseDeviceInfo(data, offset, length);

		case (byte) RCV_HEALTH: // health
			return parseHealth(data, offset, length);

		case (byte) RCV_SCAN: // scan and force-scan
			if (parseScan(data, offset, length)) {
				scanning = true;
				return true;
			}
			break;
		default:
			System.out.printf("Unknown packet type = 0x%02x\n", type);
		}
		return false;
	}

	protected boolean parseHealth(byte[] data, int offset, int length) {
		if (length != 3) {
			System.out.println("  bad health packet");
			return false;
		}

		mLatestHealth.status = data[offset] & 0xFF;
		mLatestHealth.errorCode = (data[offset + 1] & 0xFF) | ((data[offset + 2] & 0xFF) << 8);

		mListener.handleDeviceHealth(mLatestHealth);
		return true;
	}

	protected boolean parseDeviceInfo(byte[] data, int offset, int length) {
		if (length != 20) {
			System.out.println("  bad device info");
			return false;
		}

		mLatestDeviceInfo.model = data[offset] & 0xFF;
		mLatestDeviceInfo.firmwareMinor = data[offset + 1] & 0xFF;
		mLatestDeviceInfo.firmwareMajor = data[offset + 2] & 0xFF;
		mLatestDeviceInfo.hardware = data[offset + 3] & 0xFF;

		for (int i = 0; i < 16; i++) {
			mLatestDeviceInfo.serialNumber[i] = data[offset + 4 + i];
		}

		mListener.handleDeviceInfo(mLatestDeviceInfo);
		return true;
	}

	protected boolean parseScan(byte[] data, int offset, int length) {

		if (length != 5)
			return false;

		byte b0 = data[offset];
		byte b1 = data[offset + 1];

		boolean start0 = (b0 & 0x01) == 1;
		boolean start1 = (b0 & 0x02) >> 1 == 1;

		if (start0 == start1)
			return false;

		if ((b1 & 0x01) != 1)
			return false;

		var measurement = new RPLidarMeasurement();
		measurement.timestamp = System.currentTimeMillis();
		measurement.start = start0;
		measurement.quality = (b0 & 0xFF) >> 2;
		int angle = ((b1 & 0xFF) | ((data[offset + 2] & 0xFF) << 8)) >> 1;
		int distance = ((data[offset + 3] & 0xFF) | ((data[offset + 4] & 0xFF) << 8));

		measurement.angle = angle / 64f;
		measurement.distance = (distance / 4f) / 1000f;

		mListener.handleMeasurement(measurement);
		return true;
	}

	public void setVerbose(boolean verbose) {
		this.mVerbose = verbose;
	}

	/**
	 * Thread which reads in coming serial data from the LIDAR
	 */
	public class ReadSerialThread implements Runnable {

		byte data[] = new byte[1024 * 2];
		int size = 0;

		private AtomicBoolean run;

		public ReadSerialThread() {
			run = new AtomicBoolean(true);
		}

		public synchronized void requestStop() {
			run = new AtomicBoolean(false);
		}

		@Override
		public void run() {
			while (run.get()) {
				try {
					if (mInStream.available() > 0) {
						int totalRead = mInStream.read(data, size, data.length - size);

						size += totalRead;

						int used = parseData(data, size);

						// shift the buffer over by the amount read
						for (int i = 0; i < size - used; i++) {
							data[i] = data[i + used];
						}
						size -= used;
					}

					Thread.sleep(1);
				} catch (Exception e) {
					System.out.println(e.getLocalizedMessage());
					e.printStackTrace();
				}
			}
		}
	}
}
