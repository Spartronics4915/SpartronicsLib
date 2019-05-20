package com.spartronics4915.lib.statemachine;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;

public class TestCommandStateMachine extends CommandStateMachine
{
    private class TestCommand extends Command
    {

        @Override
        protected void execute()
        {
            mHasCommandRun = true;
        }

        @Override
        protected boolean isFinished()
        {
            return false;
        }
    }

    private volatile boolean mHasEntryCodeRun = false;
    private volatile boolean mHasExitCodeRun = false;

    private volatile boolean mHasCommandRun = false;
    private volatile boolean mHasLambdaRun = false;

    public TestCommandStateMachine()
    {
    }

    @Test
    public void TestScheduler() throws InterruptedException
    {

        State testCommandState = addState(new TestCommand());
        testCommandState.addCode(() -> mHasLambdaRun = true);
        testCommandState.addEntryCode(() -> mHasEntryCodeRun = true);
        testCommandState.addExitCode(() -> mHasExitCodeRun = true);
        testCommandState.whenTimeElapsed(finishedState(), 0.01);
        setInitialState(testCommandState);

        double startTime = Timer.getFPGATimestamp();
        while (Timer.getFPGATimestamp() - startTime <= 1000)
        {
            /*
             * XXX: This doesn't work because scheduler.run and Timer.getFPGATimestamp
             * _don't_ go well together, and getFPGATimestamp exhibits very strange
             * functonality
             */
            Scheduler.getInstance().run();
        }

        assertTrue(mHasCommandRun);
        assertTrue(mHasLambdaRun);
        assertTrue(mHasEntryCodeRun);
        assertTrue(mHasExitCodeRun);
    }
}
