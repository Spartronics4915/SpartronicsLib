package com.spartronics4915.lib.statemachine;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 * A state class holds a set of {@link edu.wpi.first.wpilibj.command.Command
 * Command}s, a list of exit conditions, and arbitrary code to run when the
 * state starts, exits, or while the state is running.
 * 
 * States are held and run by a {@link CommandStateMachine}. All code is
 * scheduled by the CommandStateMachine, except the Commands, which are
 * scheduled by the wpilib {@link edu.wpi.first.wpilibj.command.Scheduler
 * Scheduler}.
 */
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
     * with the {@link State#whenAnyFinished(State) whenAnyFinished} and
     * {@link State#whenAllFinished(State) whenAllFinished} methods.
     * 
     * @param command Command to run while the state machine is running
     */
    public void addCommand(Command command)
    {
        mCommands.add(command);
    }

    // Design TODO: addCommand that takes a Supplier<Command>?

    /**
     * Adds arbitrary code to be periodically executed while in this state.
     * 
     * @param action Code to run periodically while in the state (you probably want
     *               to pass a lambda here)
     */
    public void addCode(Runnable action)
    {
        mRunningCode.add(action);
    }

    /**
     * Adds arbitrary code to be run once every time the state is entered.
     * 
     * @param action Code to run once every time the state is entered (you probably
     *               want to pass a lambda here)
     */
    public void addEntryCode(Runnable action)
    {
        mEntryCode.add(action);
    }

    /**
     * Adds arbitrary code to be run once every time the state is exited.
     * 
     * @param action Code to run once every time the state is exited (you probably
     *               want to pass a lambda here)
     */
    public void addExitCode(Runnable action)
    {
        mExitCode.add(action);
    }

    /**
     * When an event happens, transition to another given state.
     * 
     * @param next Next state to transition to
     * @param cond When this returns true, we will transition
     */
    public void whenEvent(State next, Supplier<Boolean> cond)
    {
        mTransitionEvents.add(new TransitionEvent(next, cond));
    }

    /**
     * When all commands added to this state have finished, transition to another
     * given state.
     * 
     * @param next State to transition to when all commands have completed
     */
    public void whenAllFinished(State next)
    {
        if (mAnyFinishedNextState != null)
            throw new RuntimeException("You can't set whenAllFinished if whenAnyFinished is already set");
        mAllFinishedNextState = next;
    }

    /**
     * When any commands added to this state have finished, transition to another
     * given state.
     * 
     * @param next State to transition to when any command has completed
     */
    public void whenAnyFinished(State next)
    {
        if (mAllFinishedNextState != null)
            throw new RuntimeException("You can't set whenAnyFinished if whenAllFinished is already set");
        mAnyFinishedNextState = next;
    }

    /**
     * When a given number of seconds since transition into this state have elapsed,
     * transition to another given state.
     * 
     * @param next    State to transition into after the given time has elapsed
     * @param seconds Number of seconds from when we transitioned into this state to
     *                transition into the next state
     */
    public void whenTimeElapsed(State next, double seconds)
    {
        mTimeElapsedEvent = new TimeElapsedEvent(next, seconds);
    }

    // Design TODO: Add a whenTotalTimeElapsed (time since state machine was started)

    /**
     * @return Time elapsed since we transitioned into this state
     * @throws RuntimeException If this is called when the state is not running
     */
    public double elapsedTime()
    {
        if (mStartTime == kStartTimeUnsetValue)
            throw new RuntimeException("Can't call elapsedTime when the state is not running");
        return Timer.getFPGATimestamp() - mStartTime;
    }

    // Design TODO: Add a totalElapsedTime (see above design todo)

    /**
     * @return If all commands have finished running
     */
    public boolean isAllCommandsFinished()
    {
        for (Command c : mCommands)
        {
            if (!c.isCompleted())
                return false;
        }
        return true;
    }

    /**
     * @return If any commands have finished running
     */
    public boolean isAnyCommandsFinished()
    {
        for (Command c : mCommands)
        {
            if (c.isCompleted())
                return true;
        }
        return false;
    }

    /**
     * Check if a given command has finished running.
     * 
     * @param command The exact class that the command you want to check was
     *                instantiated from
     * @return If the given command is running or not
     */
    public boolean isCommandFinished(Class<? extends Command> command)
    {
        String className = command.getCanonicalName();
        for (Command c : mCommands)
        {
            if (c.getClass().getCanonicalName().equals(className))
                return c.isCompleted();
        }
        throw new RuntimeException("Couldn't find an instance of specified class");
    }

    /**
     * This method will be called by {@link CommandStateMachine#run()}. Do not call
     * this method yourself.
     * 
     * @param isFirstRun Is this the first run after transitioning into this state
     * @return Next state to transition into. This will be null if we should stay in
     *         this state.
     */
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
            mStartTime = kStartTimeUnsetValue;
        }

        return nextState;
    }

    private void runRunnableList(List<Runnable> list)
    {
        for (Runnable runnable : list)
            runnable.run();
    }

}
