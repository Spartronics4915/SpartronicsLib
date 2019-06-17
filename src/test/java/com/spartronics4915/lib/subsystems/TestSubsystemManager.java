package com.spartronics4915.lib.subsystems;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

public class TestSubsystemManager
{
    private boolean mHasOutputTelem = false;

    @Test
    public void testOutputTelemetry()
    {
        Subsystem sub = new Subsystem()
        {
            @Override
            public void outputTelemetry()
            {
                mHasOutputTelem = true;
            }

            @Override
            public void stop()
            {
                // No-op
            }
        };

        SubsystemManager man = SubsystemManager.getInstance();
        man.registerSubsystems(sub);
        man.outputAllTelemetry();

        assertTrue(mHasOutputTelem);
    }
}