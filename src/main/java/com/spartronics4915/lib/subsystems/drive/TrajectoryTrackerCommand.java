package com.spartronics4915.lib.subsystems.drive;

import java.util.function.Supplier;

import com.spartronics4915.lib.math.twodim.control.TrajectoryTracker;
import com.spartronics4915.lib.math.twodim.geometry.Pose2dWithCurvature;
import com.spartronics4915.lib.math.twodim.trajectory.types.TimedTrajectory;
import com.spartronics4915.lib.subsystems.estimator.RobotStateMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;

public class TrajectoryTrackerCommand extends Command {
    
    private final TrajectoryTrackerDriveBase mDriveBase;
    private final Supplier<TimedTrajectory<Pose2dWithCurvature>> mTrajectory;
    private final TrajectoryTracker mTracker;
    private final RobotStateMap mRobotPositionMap;

    public TrajectoryTrackerCommand(
        AbstractDrive driveSubsystem,
        TimedTrajectory<Pose2dWithCurvature> trajectory,
        TrajectoryTracker tracker,
        RobotStateMap robotPositionMap
    ) {
        this(driveSubsystem, driveSubsystem, trajectory, tracker, robotPositionMap);
    }

    public TrajectoryTrackerCommand(
            Subsystem driveSubsystem,
            TrajectoryTrackerDriveBase driveBase,
            TimedTrajectory<Pose2dWithCurvature> trajectorySupplier,
            TrajectoryTracker tracker,
            RobotStateMap robotPositionMap
        ) {
        this(driveSubsystem, driveBase, () -> trajectorySupplier, tracker, robotPositionMap);
    }

    public TrajectoryTrackerCommand(
            Subsystem driveSubsystem,
            TrajectoryTrackerDriveBase driveBase,
            Supplier<TimedTrajectory<Pose2dWithCurvature>> trajectorySupplier,
            TrajectoryTracker tracker,
            RobotStateMap robotPositionMap
    ) {
        mDriveBase = driveBase;
        mTrajectory = trajectorySupplier;
        mTracker = tracker;
        mRobotPositionMap = robotPositionMap;

        requires(driveSubsystem);
    }

    @Override
    protected void initialize() {
        super.initialize();

        mTracker.reset(mTrajectory.get());
    }

    @Override
    protected void execute() {
        super.execute();

        var nextState = mTracker.nextState(
            mRobotPositionMap.getFieldToVehicle(Timer.getFPGATimestamp()),
            Timer.getFPGATimestamp()
        );
        mDriveBase.setOutput(nextState);
    }

    @Override
    protected void end() {
        super.end();

        mDriveBase.zeroOutputs();
    }

    @Override
    protected boolean isFinished() {
        return mTracker.isFinished();
    }
}