package com.spartronics4915.lib.hardware.sensors;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Formatter;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;

import com.fazecast.jSerialComm.SerialPort;
import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;
import com.spartronics4915.lib.math.twodim.geometry.Translation2d;
import com.spartronics4915.lib.subsystems.estimator.RobotStateMap;

import edu.wpi.first.wpilibj.Timer;

/**
 * This is a single-file Java driver for the RPLidarA1. This driver supports
 * starting and stopping scans, getting individual points in a stream, getting
 * device-relative pointclouds, and getting robot-relative pointclouds.
 * 
 * This driver may work with the A2, but it does not use in express scan mode,
 * which means that you cannot use the A2 at its highest rated speed.
 * 
 * @author Declan Freeman-Gleason
 */
public class RPLidarA1 {

    private static enum OutgoingPacket {
        RESET(0x20), STOP(0x5A), SCAN(0x20), GET_INFO(0x50), GET_HEALTH(0xF0);

        public final byte header;

        private OutgoingPacket(int header) {
            this.header = (byte) header;
        }
    }

    private static enum IncomingPacket {
        INFO(0x04, 20), HEALTH(0x06, 3), SCAN(0x81, 5);

        public final byte header;
        public final int size;

        private IncomingPacket(int header, int size) {
            this.header = (byte) header;
            this.size = size;
        }
    }

    public static class DeviceHealth {

        public static enum HealthStatus {
            GOOD, WARNING, ERROR
        }

        public final HealthStatus status;
        public final int errorCode;

        public DeviceHealth(int status, int errorCode) {
            this.status = HealthStatus.values()[status]; // With throw if out of range
            this.errorCode = errorCode;
        }

        @Override
        public String toString() {
            return status + (status != HealthStatus.GOOD ? " (Error code " + errorCode + ")" : "");
        }
    }

    public static class DeviceInfo {
        public final int model;
        public final int firmwareMinor;
        public final int firmwareMajor;
        public final int hardwareRevision;
        public final byte[] serialNumber;

        public DeviceInfo(int model, int firmwareMinor, int firmwareMajor, int hardwareRevision, byte[] serialNumber) {
            this.model = model;
            this.firmwareMinor = firmwareMinor;
            this.firmwareMajor = firmwareMajor;
            this.hardwareRevision = hardwareRevision;
            this.serialNumber = serialNumber;
        }

        @Override
        public String toString() {
            try (var formatter = new Formatter()) {
                for (byte b : serialNumber) {
                    formatter.format("%02x", b);
                }
                return "RPLidar model " + model + ", firmware version " + firmwareMajor + "." + firmwareMinor
                        + ", hardware revision " + hardwareRevision + ", serial number " + formatter.toString();
            }
        }
    }

    /*
     * This is lowest level user-facing representation of what we get back from the
     * sensor.
     */
    public static class Measurement {
        /** Is this the start of a new scan */
        public final boolean start;
        /** Scan quality from 0 (lowest) to 255 (highest) */
        public final int quality;
        /** Angle in degrees */
        public final Rotation2d angle;
        /** Distance in meters */
        public final double distance;
        /** Timer.getFPGATimestamp() when the measurement arrived */
        public final double timestamp;

        public Measurement(boolean start, int quality, double angleDegrees, double distanceMeters,
                double timestampSeconds) {
            this.start = start;
            this.quality = quality;
            this.angle = Rotation2d.fromDegrees(angleDegrees);
            this.distance = distanceMeters;
            this.timestamp = timestampSeconds;
        }

        public Translation2d getAsPoint() {
            return new Translation2d(angle.cos() * distance, angle.sin() * distance);
        }

        public boolean isInvalid() {
            return distance == 0;
        }

        public String toString() {
            return "Starting: " + start + ", Quality: " + quality + ", Angle: " + angle + ", Distance: " + distance
                    + ", Timestamp: " + timestamp;
        }
    }

    // We have to make these aliases because Java type erasure
    public static interface MeasurementConsumer extends Consumer<Measurement> {
    }

    public static interface PointConsumer extends Consumer<Translation2d> {
    }

    public static interface PointcloudConsumer extends Consumer<List<Translation2d>> {
    }

    private static final String kPortDescription = "CP2102 USB to UART Bridge Controller";
    private static final byte kSyncByteZero = (byte) 0xA5;
    private static final byte kSyncByteOne = (byte) 0x5A;
    /** Seconds */
    private static final double kSendBlockingRetryDelay = 0.02;
    /** Seconds */
    private static final double kReadThreadDelay = 0.02;
    /** Seconds */
    private static final double kSendTimeout = 2;

    private Consumer<Measurement> mMeasurementConsumer;
    private final SerialPort mSerialPort;
    private InputStream mInStream;
    private OutputStream mOutStream;

    // We keep a large buffer as a member because the entire packet isn't always
    // available
    private byte[] mReadBuffer = new byte[2048];
    private int mEndOfDataIndex = 0;

    private Object mLastRecievedHeaderLock = new Object();
    private int mLastRecievedHeader = 0;

    private Optional<DeviceHealth> mLastDeviceHealth = Optional.empty();
    private Optional<DeviceInfo> mLastDeviceInfo = Optional.empty();

    // There are two modes:
    // 1. Request/response mode, where a each request has a response.
    // 2. Many response mode, where one request has many responses.
    //
    // The first mode is used for all request types except the scan request. The
    // scan request puts us into the second mode and we can only get out by sending
    // the stop request.
    private AtomicBoolean mInScanMode = new AtomicBoolean(false);
    private AtomicBoolean mIsStarted = new AtomicBoolean(false);

    // This list will be empty unless the user uses one of the pointcloud ctors
    private List<Translation2d> mCurrentPointcloud = new ArrayList<>();

    public RPLidarA1() {
        mMeasurementConsumer = (m) -> {};
        mSerialPort = Arrays.stream(SerialPort.getCommPorts())
                .filter((SerialPort p) -> p.getPortDescription().equals(kPortDescription) && !p.isOpen()).findFirst()
                .orElseThrow(() -> new RuntimeException("No RPLidar device found"));

        mSerialPort.setComPortParameters(115200, 8, SerialPort.ONE_STOP_BIT, SerialPort.NO_PARITY);
        mSerialPort.setFlowControl(SerialPort.FLOW_CONTROL_DISABLED);
        mSerialPort.openPort();

        mInStream = mSerialPort.getInputStream();
        mOutStream = mSerialPort.getOutputStream();

        new Thread(this::readData).start();

        stop();
    }

    /**
     * @param measurementConsumer A callback to recieve measurements. Note that this
     *                            will be called from an independent read thread.
     */
    public void setCallback(MeasurementConsumer measurementConsumer) {
        synchronized (mMeasurementConsumer) {
            mMeasurementConsumer = measurementConsumer;
        }
    }

    /**
     * @param pointConsumer A callback that accepts single sensor-relative points,
     *                      in meters. Note that this will be called from an
     *                      independant read thread.
     */
    public void setCallback(PointConsumer pointConsumer) {
        setCallback((Measurement m) -> {
            pointConsumer.accept(m.getAsPoint());
        });
    }

    /**
     * @param pointcloudConsumer A callback that accepts a field-relative
     *                           pointcloud, with all coordinates in meters.
     * @param robotStateMap      A robot state map to transform the pointcloud with.
     * @param vehicleToLidar     Transformation from the vehicle to the lidar sensor
     *                           (i.e. the sensor's offset from the vehicle's
     *                           center).
     */
    public void setCallback(PointcloudConsumer pointcloudConsumer, RobotStateMap robotStateMap, Pose2d vehicleToLidar) {
        setCallback((Measurement m) -> {
            if (m.start && mCurrentPointcloud.size() > 0) {
                pointcloudConsumer.accept(new ArrayList<>(mCurrentPointcloud));
                mCurrentPointcloud.clear();
            }
            Translation2d point = m.getAsPoint();
            point = robotStateMap.getFieldToVehicle(m.timestamp).transformBy(vehicleToLidar)
                    .transformBy(new Pose2d(point, new Rotation2d())).getTranslation();
            mCurrentPointcloud.add(point);
        });
    }

    /**
     * @param pointcloudConsumer A callback that accepts a sensor-relative
     *                           pointcloud, with all coordinates in meters. This is
     *                           susceptible to motion smear if you move during a
     *                           scan.
     */
    public void setCallback(PointcloudConsumer pointcloudConsumer) {
        setCallback(pointcloudConsumer, new RobotStateMap(), new Pose2d());
    }

    public void start() {
        mInScanMode.set(false);
        mIsStarted.set(true);

        mSerialPort.clearDTR();
        sendData(OutgoingPacket.SCAN, IncomingPacket.SCAN, kSendTimeout);
    }

    public void stop() {
        mInScanMode.set(false);
        mIsStarted.set(false);

        sendData(OutgoingPacket.RESET);
        Timer.delay(0.002); // Docs say to sleep 2ms after a reset
        sendData(OutgoingPacket.STOP);
        Timer.delay(0.02); // This is also advised by the docs
        mSerialPort.setDTR();
    }

    public Optional<DeviceHealth> getHealth() {
        sendData(OutgoingPacket.GET_HEALTH, IncomingPacket.HEALTH, kSendTimeout);
        synchronized (mLastDeviceHealth) {
            return mLastDeviceHealth;
        }
    }

    public Optional<DeviceInfo> getInfo() {
        sendData(OutgoingPacket.GET_INFO, IncomingPacket.INFO, kSendTimeout);
        synchronized (mLastDeviceInfo) {
            return mLastDeviceInfo;
        }
    }

    private void readData() {
        try {
            // Should we allow users to stop the read thread?
            while (true) {
                if (mInStream.available() > 0 && mIsStarted.get()) {
                    // Note that there is a chance packets get truncated if we exceed the read
                    // buffer.
                    // This shouldn't happen in practice though because messages aren't greater than
                    // 84 bytes.
                    int totalRead = mInStream.read(mReadBuffer, mEndOfDataIndex, mReadBuffer.length - mEndOfDataIndex);
                    mEndOfDataIndex += totalRead;

                    // Here we parse the message and shift everything over such that the bytes we
                    // just read are cleared from the buffer.
                    int totalUsed = parseData();
                    for (int i = 0; i < mEndOfDataIndex - totalUsed; i++) {
                        mReadBuffer[i] = mReadBuffer[i + totalUsed];
                    }
                    mEndOfDataIndex -= totalUsed;
                } else {
                    Timer.delay(kReadThreadDelay);
                }
            }
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * @return The number of bytes that were parsed and used
     */
    private int parseData() {
        int offset = 0;
        while (true) {
            if (mInScanMode.get()) {
                // There are no more complete scan packets to parse so we just return
                if (offset + IncomingPacket.SCAN.size > mEndOfDataIndex) {
                    return offset;
                }

                if (parseScan(offset, IncomingPacket.SCAN.size)) {
                    offset += IncomingPacket.SCAN.size;
                } else {
                    // Bad packet
                    System.err.println("Got a bad scan packet");
                    offset += 1;
                }
            } else {
                // Check if we have consumed enough bytes to get to the response length field
                // (which is 6 bytes in: 2 bytes for the start flags and 4 bytes for the length
                // field itself)
                if (offset + 6 > mEndOfDataIndex) {
                    return offset;
                }

                if (mReadBuffer[offset] == kSyncByteZero && mReadBuffer[offset + 1] == kSyncByteOne) {
                    // Packet length is a 30-bit unsigned integer. The 2 least significant bytes
                    // after those 30 bytes give the current send mode.

                    // First convert the bytes to an integer
                    int packetLength = ByteBuffer.wrap(Arrays.copyOfRange(mReadBuffer, offset + 2, offset + 6))
                            .order(ByteOrder.LITTLE_ENDIAN).getInt();
                    // Then mask off the first two (least significant) bits
                    packetLength &= 0x3FFFFFFF;

                    // Get the packet header
                    byte header = mReadBuffer[offset + 6];

                    // 7 is the first two sync bytes, plus the 4 bytes for packet length/send mode,
                    // plus 1 byte for the data type (header) of the packet.
                    // Adding this to offset gives us the starting address of the actual packet
                    // data.
                    int packetDataOffset = 7;

                    if (offset + packetDataOffset + packetLength > mEndOfDataIndex) {
                        return offset;
                    }

                    if (parsePacket(offset + packetDataOffset, packetLength, header)) {
                        synchronized (mLastRecievedHeaderLock) {
                            // AND wiht 0xFF is to convert an unsigned to signed integer
                            mLastRecievedHeader = header & 0xFF;
                            offset += packetDataOffset + packetLength;
                        }
                    } else {
                        // The packet was bad; this sometimes happens on startup
                        offset += 2;
                    }
                } else {
                    // We should only get here on startup; getting to this point in other situations
                    // means that we probably got a malformed packet, or that we parsed a packet
                    // wrong which screwed up our offset
                    offset++;
                }
            }
        }
    }

    private boolean parsePacket(int offset, int length, byte header) {
        // Can't do a switch because these aren't constant expressions
        if (header == IncomingPacket.INFO.header) {
            return parseDeviceInfo(offset, length);
        } else if (header == IncomingPacket.HEALTH.header) {
            return parseDeviceHealth(offset, length);
        } else if (header == IncomingPacket.SCAN.header) {
            // We only get here when we recieve the initial scan packet
            if (parseScan(offset, length)) {
                mInScanMode.set(true);
                return true;
            }
        }
        return false;
    }

    private boolean parseScan(int offset, int length) {
        if (length != IncomingPacket.SCAN.size) {
            return false;
        }

        // The least significant bit of byte zero indicates if this is thes start of a
        // new scan.
        // The second to least significant bit should be the oppositive of the least
        // significant bit.
        // The remaining 6 bits are an unsigned integer that represents the quality of
        // the point.
        byte byteZero = mReadBuffer[offset];
        // The least significant bit of byte one should always be 1; it is a check bit.
        // The remaining 6 bits are part of a fixed-point number representing the angle
        // of the sensor at this point.
        byte byteOne = mReadBuffer[offset + 1];

        boolean isStart = (byteZero & 0x01) == 1;
        boolean notIsStart = (byteZero & 0x02) >> 1 == 1;

        if (isStart == notIsStart) {
            return false;
        }

        if ((byteOne & 0x01) != 1) {
            // This is probably not a scan packet, which means we have the wrong offset
            // That only really happens on startup
            return false;
        }

        double timestamp = Timer.getFPGATimestamp();
        int quality = (byteZero & 0xFF) >> 2; // Convert to signed int and get rid of the last two bits
        int angle = ((byteOne & 0xFF) | ((mReadBuffer[offset + 2] & 0xFF) << 8)) >> 1; // Convert to signed int and extract from multiple bytes
        int distance = ((mReadBuffer[offset + 3] & 0xFF) | ((mReadBuffer[offset + 4] & 0xFF) << 8)); // Same as above

        // We negate because the angle they provide is counter clockwise positive
        double angleDegrees = -1 * angle / 64d;
        double distanceMeters = (distance / 4d) / 1000d;

        var m = new Measurement(isStart, quality, angleDegrees, distanceMeters, timestamp);
        synchronized (mMeasurementConsumer) {
            mMeasurementConsumer.accept(m);
        }

        return true;
    }

    private boolean parseDeviceHealth(int offset, int length) {
        if (length != IncomingPacket.HEALTH.size) {
            System.err.println("Bad health packet");
            return false;
        }

        // AND by 0xFF is to convert unsigned integers to signed integers
        // The bit shift and OR is to convert the little endian bytes to integers (this
        // can be dne with a ByteBuffer, but it's slower and much less concise)
        DeviceHealth h = new DeviceHealth(mReadBuffer[offset] & 0xFF,
                (mReadBuffer[offset + 1] & 0xFF) | ((mReadBuffer[offset + 2] & 0xFF) << 8));
        synchronized (mLastDeviceHealth) {
            mLastDeviceHealth = Optional.of(h);
        }
        return true;
    }

    private boolean parseDeviceInfo(int offset, int length) {
        if (length != IncomingPacket.INFO.size) {
            System.err.println("Bad device info packet");
            return false;
        }

        byte[] serialNumber = new byte[16];
        for (int i = 0; i < serialNumber.length; i++) {
            serialNumber[i] = mReadBuffer[offset + i + 4];
        }

        synchronized (mLastDeviceInfo) {
            // AND by 0xFF to convert to signed integer from unsigned
            mLastDeviceInfo = Optional.of(new DeviceInfo(mReadBuffer[offset] & 0xFF, mReadBuffer[offset + 1] & 0xFF,
                    mReadBuffer[offset + 2] & 0xFF, mReadBuffer[offset + 3] & 0xFF, serialNumber));
        }
        return true;
    }

    /**
     * Sends data and blocks until the expected packet is recieved, or the timeout
     * is hit.
     */
    private void sendData(OutgoingPacket packet, IncomingPacket expected, double timeoutSeconds) {
        sendData(packet);

        synchronized (mLastRecievedHeaderLock) {
            double endTime = Timer.getFPGATimestamp() + timeoutSeconds;
            while (endTime >= Timer.getFPGATimestamp() && mLastRecievedHeader != expected.header) {
                Timer.delay(kSendBlockingRetryDelay);
            }
        }
    }

    /**
     * Sends a packet without blocking.
     */
    private void sendData(OutgoingPacket packet) {
        try {
            mOutStream.write(new byte[] { kSyncByteZero, packet.header });
            mOutStream.flush();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

}