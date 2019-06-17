package com.spartronics4915.lib.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * This class is very simple. Its only job is to call
 * {@link Subsystem#outputTelemetry()} on all registered subsystems.
 */
public class SubsystemManager
{

    private static final SubsystemManager sInstance = new SubsystemManager();

    public static SubsystemManager getInstance()
    {
        return sInstance;
    }

    private SubsystemManager()
    {
    }

    public List<Subsystem> mSubsystems = new ArrayList<>();

    public void registerSubsystems(Subsystem... subsystems)
    {
        mSubsystems.addAll(Arrays.asList(subsystems));
    }

    public void outputAllTelemetry()
    {
        for (Subsystem subsystem : mSubsystems)
            subsystem.outputTelemetry();
    }
}
