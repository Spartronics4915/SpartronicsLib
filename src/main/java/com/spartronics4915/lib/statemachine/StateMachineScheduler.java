package com.spartronics4915.lib.statemachine;

import edu.wpi.first.wpilibj.command.Scheduler;

public final class StateMachineScheduler
{

    private static StateMachineScheduler sInstance = null;

    private Scheduler mScheduler;
    private CommandStateMachine mStateMachine = null;
    private boolean mIsFirstRun = true;

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

    /**
     * This method calls {@link CommandStateMachine#run()} and then calls
     * {@link edu.wpi.first.wpilibj.command.Scheduler#run()
     * Scheduler.getInstance().run()}. Put this in your {@code teleopPeriodic} and
     * {@code autonomousPeriodic} methods.
     */
    public void run()
    {
        if (mIsFirstRun)
            mScheduler.enable();
        if (mStateMachine != null)
            mStateMachine.run(mIsFirstRun);
        mScheduler.run();

        mIsFirstRun = false;
    }

    /**
     * Resets the scheduler in a way that will fully reset the underlying state
     * machine and commands.
     * 
     * Call this in your {@code disabledInit} methods.
     */
    public void reset()
    {
        mScheduler.disable();
        mIsFirstRun = true;
    }
}
