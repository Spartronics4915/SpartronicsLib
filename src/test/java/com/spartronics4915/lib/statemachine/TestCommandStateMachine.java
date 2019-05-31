package com.spartronics4915.lib.statemachine;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;

public class TestCommandStateMachine
{
    private class TestCommand extends Command
    {

        public TestCommand()
        {
            setRunWhenDisabled(true);
        }

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

    private boolean mHasEntryCodeRun = false;
    private boolean mHasExitCodeRun = false;

    private boolean mHasCommandRun = false;
    private boolean mHasLambdaRun = false;

    public TestCommandStateMachine()
    {
    }
    
    @Test
    public void testNoSetInitial()
    {
        CommandStateMachine csm = new CommandStateMachine() {
            @Override
            public void setup() {
                State testCommandState = addState(new TestCommand());
                // Note that setInitialState is _not_ called
                // That means we should throw
            }
        };
        StateMachineScheduler.getInstance().setStateMachine(csm);
        assertThrows(RuntimeException.class, () -> StateMachineScheduler.getInstance().run());
    }

    @Test
    public void testBadStateCalls()
    {
        new CommandStateMachine() {
            @Override
            public void setup() {
                State testCommandState = addState(new TestCommand());
                State someOtherState = addState();
                setInitialState(testCommandState);

                // Can't do elapsedTime when we haven't run yet
                assertThrows(RuntimeException.class, () -> testCommandState.elapsedTime());
                assertThrows(RuntimeException.class, () -> testCommandState.isCommandFinished(Command.class)); // This command isn't held by this state, so this should throw
                
                assertDoesNotThrow(() -> testCommandState.whenAllFinished(someOtherState));
                assertThrows(RuntimeException.class, () -> testCommandState.whenAnyFinished(someOtherState)); // Can't do whenAllFinished and whenAnyFinished
                
                testCommandState.whenAllFinished(null);
                assertDoesNotThrow(() -> testCommandState.whenAnyFinished(someOtherState));
                assertThrows(RuntimeException.class, () -> testCommandState.whenAllFinished(someOtherState)); // Can't do whenAllFinished and whenAnyFinished
            }
        };
    }

    @Test
    public void testScheduler() throws InterruptedException
    {
        CommandStateMachine csm = new CommandStateMachine() {
            @Override
            public void setup() {
                State testCommandState = addState(new TestCommand());
                testCommandState.addCode(() -> mHasLambdaRun = true);
                testCommandState.addEntryCode(() -> mHasEntryCodeRun = true);
                testCommandState.addExitCode(() -> mHasExitCodeRun = true);
                testCommandState.whenTimeElapsed(finishedState(), 0);
                setInitialState(testCommandState);
            }
        };

        StateMachineScheduler.getInstance().setStateMachine(csm);

        double startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime <= 5000 || !mHasExitCodeRun)
        {
            /*
             * Don't use Timer.getFPGATimestamp here... It does something _weird_.
             */
            StateMachineScheduler.getInstance().run();
        }

        assertTrue(mHasCommandRun);
        assertTrue(mHasLambdaRun);
        assertTrue(mHasEntryCodeRun);
        assertTrue(mHasExitCodeRun);
    }
}
