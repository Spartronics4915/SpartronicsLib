package com.spartronics4915.lib.statemachine;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.command.Scheduler;

public final class StateMachineScheduler
{

    private static StateMachineScheduler sInstance = null;

    private Scheduler mScheduler;

    private CommandStateMachine mStateMachine = null;
    private boolean mHasBeenStopped = true;

    private List<Loop> mPersistentLoops = new ArrayList<>();

    public static StateMachineScheduler getInstance()
    {
        if (sInstance == null)
            sInstance = new StateMachineScheduler();
        return sInstance;
    }

    private StateMachineScheduler()
    {
        mScheduler = Scheduler.getInstance();
    }

    public void setStateMachine(CommandStateMachine csm)
    {
        mStateMachine = csm;
    }

    public void addPersistentLoop(Loop l)
    {
        mPersistentLoops.add(l);
    }

    /**
     * This method calls {@link CommandStateMachine#run()}
     * and then calls {@link edu.wpi.first.wpilibj.command.Scheduler#run()
     * Scheduler.getInstance().run()}. Put this in your {@code teleopPeriodic} and
     * {@code autonomousPeriodic} methods.
     */
    public void run()
    {
        if (mHasBeenStopped)
            mScheduler.enable();

        for (Loop persistLoop : mPersistentLoops)
            persistLoop.run(mHasBeenStopped);

        if (mStateMachine != null)
            mStateMachine.run(mHasBeenStopped);

        mScheduler.run();

        mHasBeenStopped = false;
    }

    /**
     * Stops the scheduler in a way that will fully reset the underlying state
     * machine and commands.
     * 
     * Call this in your {@code disabledInit} methods.
     */
    public void stop()
    {
        mScheduler.disable();
        mHasBeenStopped = true;
    }
}
