package com.spartronics4915.lib.subsystems.drive;

import java.util.Set;
import java.util.function.Supplier;

import com.spartronics4915.lib.math.twodim.control.TrajectoryTracker;
import com.spartronics4915.lib.math.twodim.geometry.Pose2dWithCurvature;
import com.spartronics4915.lib.math.twodim.trajectory.types.TimedTrajectory;
import com.spartronics4915.lib.subsystems.estimator.RobotStateMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class TrajectoryTrackerCommand implements Command {

    private final Subsystem mSubsystemToRequire;
    private final TrajectoryTrackerDriveBase mDriveBase;
    private final Supplier<TimedTrajectory<Pose2dWithCurvature>> mTrajectory;
    private final TrajectoryTracker mTracker;
    private final RobotStateMap mRobotPositionMap;

    public TrajectoryTrackerCommand(AbstractDrive driveSubsystem, TimedTrajectory<Pose2dWithCurvature> trajectory,
            TrajectoryTracker tracker, RobotStateMap robotPositionMap) {
        this(driveSubsystem, driveSubsystem, trajectory, tracker, robotPositionMap);
    }

    public TrajectoryTrackerCommand(Subsystem driveSubsystem, TrajectoryTrackerDriveBase driveBase,
            TimedTrajectory<Pose2dWithCurvature> trajectorySupplier, TrajectoryTracker tracker,
            RobotStateMap robotPositionMap) {
        this(driveSubsystem, driveBase, () -> trajectorySupplier, tracker, robotPositionMap);
    }

    public TrajectoryTrackerCommand(Subsystem driveSubsystem, TrajectoryTrackerDriveBase driveBase,
            Supplier<TimedTrajectory<Pose2dWithCurvature>> trajectorySupplier, TrajectoryTracker tracker,
            RobotStateMap robotPositionMap) {
        mSubsystemToRequire = driveSubsystem;
        mDriveBase = driveBase;
        mTrajectory = trajectorySupplier;
        mTracker = tracker;
        mRobotPositionMap = robotPositionMap;
    }

    @Override
    public void initialize() {
        mTracker.reset(mTrajectory.get());
    }

    @Override
    public void execute() {
        var nextState = mTracker.nextState(mRobotPositionMap.getFieldToVehicle(Timer.getFPGATimestamp()),
                Timer.getFPGATimestamp());
        mDriveBase.setOutput(nextState);
    }

    @Override
    public void end(boolean interrupted) {
        mDriveBase.zeroOutputs();
    }

    @Override
    public boolean isFinished() {
        return mTracker.isFinished();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(mSubsystemToRequire);
    }
}