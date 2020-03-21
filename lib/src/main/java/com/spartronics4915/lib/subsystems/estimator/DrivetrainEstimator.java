package com.spartronics4915.lib.subsystems.estimator;

import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;
import com.spartronics4915.lib.util.Logger;

import edu.wpi.first.wpilibj.estimator.ExtendedKalmanFilter;
import edu.wpi.first.wpilibj.math.StateSpaceUtils;
import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N3;
import edu.wpi.first.wpiutil.math.numbers.N6;

/**
 * Wraps the Extended Kalman Filter I wrote and contributed to the prerelease version of WPILibStateSpace to fuse encoder odometry, VSLAM, and PnP to localize the robot.
 * 
 * We also do latency compensation for the vision.
 * 
 * Our state-space system is:
 *
 * x = [[x, y, dtheta]]^T in the field coordinate system
 *
 * u = [[v_l, v_r, theta]]^T Not actual inputs, but the dynamics math required
 * to use actual inputs like voltage is gross.
 *
 * y = [[x_s, y_s, theta_s, x_r, y_r, theta_r]]^T All the subscript s ones are
 * measurements from vSLAM, while the sub r ones are from retroreflective tape
 * vision.
 */
public class DrivetrainEstimator
{

    private static final double kNominalDt = 0.01;
    private static final int kMaxPastObserverStates = 200;

    private final ExtendedKalmanFilter<N3, N3, N3> mObserver;
    private final ExtendedKalmanFilter<N3, N3, N6> mVisionObserver;
    private final Matrix<N6, N1> mMeasurementStdDevs;

    private final Pose2d mBeginningRobotPose;
    private final double mSlamStdDevsPerMeter;

    private final TreeMap<Double, ObserverState> mPastObserverStates;

    public DrivetrainEstimator(Matrix<N3, N1> stateStdDevs, Matrix<N6, N1> measurementStdDevs,
        double slamStdDevsPerMeter, Pose2d beginningRobotPose)
    {
        mObserver = new ExtendedKalmanFilter<N3, N3, N3>(Nat.N3(), Nat.N3(), Nat.N3(), this::f,
            (x, u) -> x, stateStdDevs,
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(measurementStdDevs.get(0, 0),
                measurementStdDevs.get(1, 0), measurementStdDevs.get(2, 0)),
            false, kNominalDt);
        mVisionObserver = new ExtendedKalmanFilter<>(Nat.N3(), Nat.N3(), Nat.N6(), this::f, this::h,
            stateStdDevs, measurementStdDevs, false, kNominalDt);
        mMeasurementStdDevs = measurementStdDevs;

        mBeginningRobotPose = beginningRobotPose;
        mSlamStdDevsPerMeter = slamStdDevsPerMeter;

        mPastObserverStates = new TreeMap<>();
    }

    private final Matrix<N3, N1> f(Matrix<N3, N1> x, Matrix<N3, N1> u)
    {
        // Diff drive forward kinematics:
        // v_c = (v_l + v_r) / 2
        var newPose = new Pose2d(x.get(0, 0), x.get(1, 0), new Rotation2d(x.get(2, 0))).exp(
                new Twist2d((u.get(0, 0) + u.get(1, 0)) / 2, 0.0, u.get(2, 0))
        );

        return new MatBuilder<>(Nat.N3(), Nat.N1()).fill(newPose.getTranslation().getX(),
            newPose.getTranslation().getY(), x.get(2, 0) + u.get(2, 0));
    }

    private final Matrix<N6, N1> h(Matrix<N3, N1> x, Matrix<N3, N1> u)
    {
        return new MatBuilder<>(Nat.N6(), Nat.N1()).fill(x.get(0, 0), x.get(1, 0), x.get(2, 0),
            x.get(0, 0), x.get(1, 0), x.get(2, 0));
    }

    public synchronized void addVisionMeasurement(Pose2d visionRobotPose, double timestampSeconds)
    {
        var low = mPastObserverStates.floorEntry(timestampSeconds);
        var high = mPastObserverStates.ceilingEntry(timestampSeconds);

        Map.Entry<Double, ObserverState> closestEntry = null;
        if (low != null && high != null)
        {
            closestEntry = Math.abs(timestampSeconds - low.getKey()) < Math
                .abs(timestampSeconds - high.getKey())
                    ? low
                    : high;
        }
        else
        {
            closestEntry = low != null ? low : high;
        }

        if (closestEntry == null)
        {
            Logger.warning("Observer state map was empty; ignorning vision update");
            return;
        }

        var tailMap = mPastObserverStates.tailMap(closestEntry.getKey(), true);
        for (var st : tailMap.values())
        {
            if (visionRobotPose != null)
            {
                mObserver.setP(st.errorCovariances);
                mObserver.setXhat(st.xHat);
            }
            update(st.slamMeasurements, visionRobotPose, st.inputs);

            visionRobotPose = null;
        }
    }

    /**
     * @param slamRobotPose The robot pose just given by the vSLAM camera.
     * @param 
     * 
     * @return The estimated pose of the robot.
     */
    public synchronized Pose2d update(Pose2d slamRobotPose, double dleftMeters, double drightMeters,
        double dthetaRadians, double timeSeconds)
    {
        var u = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(dleftMeters, drightMeters, dthetaRadians);
        mPastObserverStates.put(timeSeconds, new ObserverState(mObserver, u, slamRobotPose));

        if (mPastObserverStates.size() > kMaxPastObserverStates)
        {
            mPastObserverStates.remove(mPastObserverStates.firstKey());
        }

        return update(slamRobotPose, null, u);
    }

    private Pose2d update(Pose2d slamRobotPose, Pose2d visionRobotPose, Matrix<N3, N1> u)
    {
        double distFromBeginningMeters = slamRobotPose.getTranslation().getDistance(mBeginningRobotPose.getTranslation());

        if (visionRobotPose != null)
        {
            mVisionObserver.setP(mObserver.getP());
            mVisionObserver.setXhat(mObserver.getXhat());

            var measStdDevs = new MatBuilder<>(Nat.N6(), Nat.N1()).fill(
                mSlamStdDevsPerMeter * distFromBeginningMeters,
                mSlamStdDevsPerMeter * distFromBeginningMeters,
                mSlamStdDevsPerMeter * distFromBeginningMeters,
                mMeasurementStdDevs.get(3, 0),
                mMeasurementStdDevs.get(4, 0),
                mMeasurementStdDevs.get(5, 0));
            var contR = StateSpaceUtils.makeCovMatrix(Nat.N6(), measStdDevs);
            var discR = StateSpaceUtils.discretizeR(contR, kNominalDt);

            var y = new MatBuilder<>(Nat.N6(), Nat.N1()).fill(slamRobotPose.getTranslation().getX(),
                slamRobotPose.getTranslation().getY(), slamRobotPose.getRotation().getRadians(),
                visionRobotPose.getTranslation().getX(), visionRobotPose.getTranslation().getY(),
                visionRobotPose.getRotation().getRadians());

            mVisionObserver.correct(Nat.N6(), u, y, this::h, discR);
            mVisionObserver.predict(u, kNominalDt);

            mObserver.setP(mVisionObserver.getP());
            mObserver.setXhat(mVisionObserver.getXhat());
        }
        else
        {
            var y = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(slamRobotPose.getTranslation().getX(),
                slamRobotPose.getTranslation().getY(), slamRobotPose.getRotation().getRadians());

            var measStdDevs = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(
                mSlamStdDevsPerMeter * distFromBeginningMeters,
                mSlamStdDevsPerMeter * distFromBeginningMeters,
                mSlamStdDevsPerMeter * distFromBeginningMeters);
            var contR = StateSpaceUtils.makeCovMatrix(Nat.N3(), measStdDevs);
            var discR = StateSpaceUtils.discretizeR(contR, kNominalDt);

            mObserver.correct(Nat.N3(), u, y, (x, _u) -> x, discR);
            mObserver.predict(u, kNominalDt);
        }

        return new Pose2d(mObserver.getXhat(0), mObserver.getXhat(1),
            new Rotation2d(mObserver.getXhat(2)));
    }

    private static class ObserverState
    {
        public final Matrix<N3, N1> xHat;
        public final Matrix<N3, N3> errorCovariances;
        public final Matrix<N3, N1> inputs;
        public final Pose2d slamMeasurements;

        public ObserverState(ExtendedKalmanFilter<N3, N3, N3> observer, Matrix<N3, N1> u,
            Pose2d y)
        {
            xHat = observer.getXhat();
            errorCovariances = observer.getP();

            inputs = u;
            slamMeasurements = y;
        }
    }
}
