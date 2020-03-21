package com.spartronics4915.lib.hardware;

public class CANCounter
{
    private static int sFoundCANDevices = 0;
    private static int sTotalCANDevices = 0;

    public static void addDevice(boolean errored)
    {
        sFoundCANDevices += errored ? 0 : 1;
        sTotalCANDevices++;
    }

    public static int getFoundDevices()
    {
        return sFoundCANDevices;
    }

    public static int getTotalDevices()
    {
        return sTotalCANDevices;
    }

    public static String getStatusMessage()
    {
        return sFoundCANDevices == sTotalCANDevices ? "OK"
            : sFoundCANDevices + "/" + sTotalCANDevices;
    }
}
