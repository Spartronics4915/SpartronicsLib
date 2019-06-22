package com.spartronics4915.lib.statemachine;

import java.util.List;

import edu.wpi.first.wpilibj.command.Command;

abstract public class CommandStateMachine
{

    private static final State kFinishedState = new State();

    private State mInitialState = null;
    private State mCurrentState = null;
    private boolean mIsStateNew = true;

    /**
     * Adds a new empty State to this state machine.
     * 
     * @return A new empty State class, ready for use
     */
    protected final State addState()
    {
        return new State();
    }

    /**
     * Adds a State to this state machine, with one or more given
     * {@link edu.wpi.first.wpilibj.command.Command Commands}s pre-added to the
     * State.
     * 
     * @param commands Commands to add to the new state
     * @return The new State to return
     */
    protected final State addState(Command... commands)
    {
        State state = addState();
        for (Command c : commands)
            state.addCommand(c);
        return state;
    }

    /**
     * Adds a State to this state machine, with the given list of
     * {@link edu.wpi.first.wpilibj.command.Command Commands}s pre-added to the
     * State.
     * 
     * @param commands Commands to add to the new state
     * @return The new State to return
     */
    protected final State addState(List<Command> commands)
    {
        return addState(commands.toArray(new Command[commands.size()]));
    }

    /**
     * Gets a "finished" State, which can be transitioned to when you're done with
     * your other states. Only useful if you want your state machine to stop (like
     * in an autononous mode).
     * 
     * @return A state that does nothing
     */
    protected final State finishedState()
    {
        return kFinishedState;
    }

    /**
     * Set the initial state of the state machine.
     * 
     * @param state The state to begin in when the state machine starts
     */
    protected final void setInitialState(State state)
    {
        mInitialState = state;
    }

    /**
     * This method runs the current State and transitions into new States if needed.
     * This should be called directly <strong>before</strong> wherever you call
     * {@link edu.wpi.first.wpilibj.command.Scheduler#run()
     * Scheduler.getInstance().run()}.
     * 
     * @param shouldReset Should everything be reset (this is true on the first run)
     */
    public final void run(boolean shouldReset)
    {
        if (shouldReset)
        {
            mInitialState = null;
            mCurrentState = null;
            mIsStateNew = true;
            setup();
        }

        if (mInitialState == null)
            throw new RuntimeException("You must set an initial state");
        if (mCurrentState == null)
            mCurrentState = mInitialState;

        State nextState = mCurrentState.run(mIsStateNew);
        mIsStateNew = mCurrentState != null && mCurrentState != nextState;
        if (mIsStateNew)
            mCurrentState = nextState;
    }

    /**
     * Override this method and call {@link CommandStateMachine#addState()}, etc
     * here.
     */
    protected abstract void setup();
}
