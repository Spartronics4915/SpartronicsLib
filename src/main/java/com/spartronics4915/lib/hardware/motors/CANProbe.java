/* from:
 * http://www.chiefdelphi.com/forums/showthread.php?threadid=142391&langid=2
 */
package com.spartronics4915.lib.hardware.motors;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;

import edu.wpi.first.hal.can.CANJNI;

public class CANProbe
{

    private static CANProbe sInstance = null;
    public static CANProbe getInstance()
    {
        if (sInstance == null)
        {
            sInstance = new CANProbe();
        }
        return sInstance;
    }

    private ByteBuffer targetID = ByteBuffer.allocateDirect(4);
    private ByteBuffer timeStamp = ByteBuffer.allocateDirect(4);
    private ArrayList<String> mReport = new ArrayList<String>();
    private ArrayList<Integer> mValidSRXIds = new ArrayList<Integer>();
    private ArrayList<Integer> mValidPCMIds = new ArrayList<Integer>();

    private CANProbe()
    {
        runProbe();
    }

    /** helper routine to get last received message for a given ID */
    private long checkMessage(int fullId, int deviceID)
    {
        try
        {
            targetID.clear();
            targetID.order(ByteOrder.LITTLE_ENDIAN);
            targetID.asIntBuffer().put(0, fullId | deviceID);

            timeStamp.clear();
            timeStamp.order(ByteOrder.LITTLE_ENDIAN);
            timeStamp.asIntBuffer().put(0, 0x00000000);

            CANJNI.FRCNetCommCANSessionMuxReceiveMessage(targetID.asIntBuffer(), 0x1fffffff,
                    timeStamp);

            long retval = timeStamp.getInt();
            retval &= 0xFFFFFFFF; /* undo sign-extension */
            return retval;
        }
        catch (Exception e)
        {
            return -1;
        }
    }

    public boolean validateSRXId(int id)
    {
        for (Integer a : mValidSRXIds)
        {
            if (a == id)
                return true;
        }
        return false;
    }

    public boolean validatePCMId(int id)
    {
        for (Integer a : mValidPCMIds)
        {
            if (a == id)
                return true;
        }
        return false;
    }

    /**
     * @return ArrayList of strings holding the names of devices we've found.
     */
    public ArrayList<String> getReport()
    {
        return mReport;
    }

    public int getCANDeviceCount() // skips the pdp, which has shown to be flaky
    {
        return mValidPCMIds.size() + mValidSRXIds.size();
    }

    /**
     * polls for received framing to determine if a device is present.
     * This is meant to be used once initially (and not periodically) since
     * this steals cached messages from the robot API.
     */
    private void runProbe()
    {
        /* get timestamp0 for each device */
        long pdp0_timeStamp0; // only look for PDP at '0'
        long[] pcm_timeStamp0 = new long[63];
        long[] srx_timeStamp0 = new long[63];

        pdp0_timeStamp0 = checkMessage(0x08041400, 0);
        for (int i = 0; i < 63; ++i)
        {
            pcm_timeStamp0[i] = checkMessage(0x09041400, i);
            srx_timeStamp0[i] = checkMessage(0x02041400, i);
        }

        /* wait 200ms */
        try
        {
            Thread.sleep(200);
        }
        catch (InterruptedException e)
        {
            e.printStackTrace();
        }

        /* get timestamp1 for each device */
        long pdp0_timeStamp1; // only look for PDP at '0'
        long[] pcm_timeStamp1 = new long[63];
        long[] srx_timeStamp1 = new long[63];

        pdp0_timeStamp1 = checkMessage(0x08041400, 0);
        for (int i = 0; i < 63; ++i)
        {
            pcm_timeStamp1[i] = checkMessage(0x09041400, i);
            srx_timeStamp1[i] = checkMessage(0x02041400, i);
        }

        /*
         * compare, if timestamp0 is good and timestamp1 is good,
         * and they are different, device is healthy
         */
        if (pdp0_timeStamp0 >= 0 && pdp0_timeStamp1 >= 0 &&
                pdp0_timeStamp0 != pdp0_timeStamp1)
            mReport.add("PDP 0");

        for (int i = 0; i < 63; ++i)
        {
            if (pcm_timeStamp0[i] >= 0 && pcm_timeStamp1[i] >= 0 &&
                    pcm_timeStamp0[i] != pcm_timeStamp1[i])
            {
                mReport.add("PCM " + i);
                mValidPCMIds.add(i);
            }
            if (srx_timeStamp0[i] >= 0 && srx_timeStamp1[i] >= 0 &&
                    srx_timeStamp0[i] != srx_timeStamp1[i])
            {
                mReport.add("SRX " + i);
                mValidSRXIds.add(i);
            }
        }
    }
}
