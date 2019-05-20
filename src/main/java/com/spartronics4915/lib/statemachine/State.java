package com.spartronics4915.lib.statemachine;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class State
{
    private final double kStartTimeUnsetValue = -1.0;

    private List<Runnable> mEntryCode = new ArrayList<>();
    private List<Runnable> mRunningCode = new ArrayList<>();
    private List<Command> mCommands = new ArrayList<>();
    private List<Runnable> mExitCode = new ArrayList<>();

    private List<TransitionEvent> mTransitionEvents = new ArrayList<>();
    private State mAllFinishedNextState = null;
    private State mAnyFinishedNextState = null;
    private TimeElapsedEvent mTimeElapsedEvent = null; 

    private double mStartTime = kStartTimeUnsetValue;

    private class TransitionEvent
    {
        public final State nextState;
        public final Supplier<Boolean> condition;

        public TransitionEvent(State next, Supplier<Boolean> cond)
        {
            this.nextState = next;
            this.condition = cond;
        }
    }

    private class TimeElapsedEvent
    {
        public final State nextState;
        public final double time;

        public TimeElapsedEvent(State next, double time)
        {
            this.nextState = next;
            this.time = time;
        }
    }

    /**
     * Adds a command to be run for the entire time the state machine is running.
     * The command will be interrupted if the state machine exits. You can also base
     * state machine transitions on the status of this (or all) running commands
     * with the *CommandsFinished methods.
     * 
     * @param command command to run while the state machine is running
     */
    // Design todo: Return Command for chaining?
    public void addCommand(Command command)
    {
        mCommands.add(command);
    }

    // Design todo: addCommand that takes a Supplier<Command>?

    public void addCode(Runnable action)
    {
        mRunningCode.add(action);
    }

    public void addEntryCode(Runnable action)
    {
        mEntryCode.add(action);
    }

    public void addExitCode(Runnable action)
    {
        mExitCode.add(action);
    }

    public void whenEvent(State next, Supplier<Boolean> cond)
    {
        mTransitionEvents.add(new TransitionEvent(next, cond));
    }

    public void whenAllFinished(State next)
    {
        if (mAnyFinishedNextState != null)
            throw new RuntimeException("You can't set whenAllFinished if whenAnyFinished is already set");
        mAllFinishedNextState = next;
    }

    public void whenAnyFinished(State next)
    {
        if (mAllFinishedNextState != null)
            throw new RuntimeException("You can't set whenAnyFinished if whenAllFinished is already set");
        mAnyFinishedNextState = next;
    }

    public void whenTimeElapsed(State next, double seconds)
    {
        mTimeElapsedEvent = new TimeElapsedEvent(next, seconds);
    }

    // Design todo: Add a whenTotalTimeElapsed (time since state machine was started)

    public double elapsedTime()
    {
        if (mStartTime == kStartTimeUnsetValue)
            throw new RuntimeException("Can't call elapsedTime before the State has been run");
        return Timer.getFPGATimestamp() - mStartTime;
    }

    // Design todo: Add a totalElapsedTime (see above design todo)

    public boolean isAllCommandsFinished()
    {
        for (Command c : mCommands)
        {
            // XXX: Is isCompleted the right thing?
            if (!c.isCompleted())
                return false;
        }
        return true;
    }

    public boolean isAnyCommandsFinished()
    {
        for (Command c : mCommands)
        {
            // XXX: Is isCompleted the right thing?
            if (c.isCompleted())
                return true;
        }
        return false;
    }

    public boolean isCommandFinished(Class<? extends Command> command)
    {
        // TODO: command.getCanonicalName().equals(...)
        String className = command.getCanonicalName();
        for (Command c : mCommands)
        {
            // XXX: Is isCompleted the right thing?
            if (c.getClass().getCanonicalName().equals(className))
                return c.isCompleted();
        }
        throw new RuntimeException("Couldn't find an instance of specified class");
    }

    public State run(boolean isFirstRun)
    {
        if (isFirstRun)
        {
            mStartTime = Timer.getFPGATimestamp();
            runRunnableList(mEntryCode);
            for (Command com : mCommands)
                com.start();
        }

        runRunnableList(mRunningCode);
        
        State nextState = null;
        if (mTimeElapsedEvent != null && elapsedTime() >= mTimeElapsedEvent.time)
        {
            nextState = mTimeElapsedEvent.nextState;
        }
        else if (isAnyCommandsFinished() && mAnyFinishedNextState != null)
        {
            nextState = mAnyFinishedNextState;
        }
        else if (isAllCommandsFinished() && mAllFinishedNextState != null)
        {
            nextState = mAllFinishedNextState;
        }
        else
        {
            for (TransitionEvent evt : mTransitionEvents)
            {
                if (evt.condition.get())
                    nextState = evt.nextState;
            }
        }

        if (nextState != null)
        {
            runRunnableList(mExitCode);
            for (Command com : mCommands)
                com.cancel();
        }

        return nextState;
    }

    private void runRunnableList(List<Runnable> list)
    {
        for (Runnable runnable : list)
                runnable.run();
    }

}
