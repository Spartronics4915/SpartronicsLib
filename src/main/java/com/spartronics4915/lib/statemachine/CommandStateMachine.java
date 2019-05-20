package com.spartronics4915.lib.statemachine;

import java.util.List;

import edu.wpi.first.wpilibj.command.Command;

// TODO: Maybe make our own scheduler instead of the wpilib command scheduler?
public class CommandStateMachine extends Command
{
    private static final State kFinishedState = new State();

    private State mInitialState = null;
    private State mCurrentState = null;
    private boolean mIsStateNew = true;

    public CommandStateMachine()
    {
        setInterruptible(false);
    }

    protected final State addState()
    {
        return new State();
    }

    protected final State addState(Command... commands)
    {
        State state = addState();
        for (Command c : commands)
            state.addCommand(c);
        return state;
    }

    protected final State addState(List<Command> commands)
    {
        return addState(commands.toArray(new Command[commands.size()]));
    }

    protected final State finishedState()
    {
        return kFinishedState;
    }

    // TODO: Method chaining?
    protected final void setInitialState(State state)
    {
        mInitialState = state;
    }

    @Override
    public void execute()
    {
        System.out.println("I'm alive!");
        if (mInitialState == null)
            throw new RuntimeException("You must set an initial state");
        if (mCurrentState == null)
            mCurrentState = mInitialState;

        State nextState = mCurrentState.run(mIsStateNew);
        mIsStateNew = mCurrentState != null && mCurrentState != nextState;
        if (mIsStateNew)
            mCurrentState = nextState;
    }

    @Override
    protected final boolean isFinished()
    {
        return false;
    }
}
