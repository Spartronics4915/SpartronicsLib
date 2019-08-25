package com.spartronics4915.lib.math.twodim.physics;

import com.spartronics4915.lib.math.Util;

import java.text.DecimalFormat;
import java.util.Arrays;

/**
 * Dynamic model a differential drive robot. Note: to simplify things, this math
 * assumes the center of mass is
 * coincident with the kinematic center of rotation (e.g. midpoint of the center
 * axle).
 */
public class DifferentialDrive
{
    // All units must be SI!

    // Equivalent mass when accelerating purely linearly, in kg.
    // This is "equivalent" in that it also absorbs the effects of drivetrain inertia.
    // Measure by doing drivetrain acceleration characterization in a straight line.
    /** Kilograms */
    protected final double mMass;

    // Equivalent moment of inertia when accelerating purely angularly, in kg*m^2.
    // This is "equivalent" in that it also absorbs the effects of drivetrain inertia.
    // Measure by doing drivetrain acceleration characterization while turning in place.
    /** kg*m^2 */
    protected final double mMoi;

    // Drag torque (proportional to angular velocity) that resists turning, in N*m/rad/s
    // Empirical testing of our drivebase showed that there was an unexplained loss in torque ~proportional to angular
    // velocity, likely due to scrub of wheels.
    // NOTE: this may not be a purely linear term, and we have done limited testing, but this factor helps our model to
    // better match reality.  For future seasons, we should investigate what's going on here...
    /** N*m/rad/s */
    protected final double mAngularDrag;

    // Self-explanatory.  Measure by rolling the robot a known distance and counting encoder ticks.
    /** Meters */
    protected final double mWheelRadius; // m

    // "Effective" kinematic wheelbase radius.  Might be larger than theoretical to compensate for skid steer.  Measure
    // by turning the robot in place several times and figuring out what the equivalent wheelbase radius is.
    /** Meters */
    protected final double mEffectiveWheelbaseRadius; // m

    // Transmissions for both sides of the drive.
    protected final DCMotorTransmission mLeftTransmission;
    protected final DCMotorTransmission mRightTransmission;

    public DifferentialDrive(
            final double massKilograms,
            final double moi, // kg m^2
            final double angularDrag, // N*m/rad/s
            final double wheelRadiusMeters,
            final double effectiveWheelbaseRadiusMeters,
            final DCMotorTransmission leftTransmission,
            final DCMotorTransmission rightTransmission)
    {
        mMass = massKilograms;
        mMoi = moi;
        mAngularDrag = angularDrag;
        mWheelRadius = wheelRadiusMeters;
        mEffectiveWheelbaseRadius = effectiveWheelbaseRadiusMeters;
        mLeftTransmission = leftTransmission;
        mRightTransmission = rightTransmission;
    }

    /** 
     * @return Robot mass in kilograms
     * */
    public double mass()
    {
        return mMass;
    }

    /** 
     * @return Robot moment of inertia in kilograms*meters^2
     */
    public double moi()
    {
        return mMoi;
    }

    /**
     * @return Wheel radius in meters
     */
    public double wheelRadius()
    {
        return mWheelRadius;
    }

    /**
     * @return Effective wheel radius in meters (this is used to calculate scrub)
     */
    public double effectiveWheelbaseRadius()
    {
        return mEffectiveWheelbaseRadius;
    }

    public DCMotorTransmission leftTransmission()
    {
        return mLeftTransmission;
    }

    public DCMotorTransmission rightTransmission()
    {
        return mRightTransmission;
    }

    public WheelState getVoltagesFromkV(WheelState velocities) {
        return new WheelState(
            velocities.left / mLeftTransmission.speedPerVolt() +
                mLeftTransmission.frictionVoltage() * Math.signum(velocities.left),
            velocities.right / mRightTransmission.speedPerVolt() +
                mRightTransmission.frictionVoltage() * Math.signum(velocities.right)
        );
    }

    // Input/demand could be either velocity or acceleration...the math is the same.
    public ChassisState solveForwardKinematics(final WheelState wheelMotion)
    {
        ChassisState chassisMotion = new ChassisState();
        chassisMotion.linear = mWheelRadius * (wheelMotion.right + wheelMotion.left) / 2.0;
        chassisMotion.angular = mWheelRadius * (wheelMotion.right - wheelMotion.left) / (2.0 *
                mEffectiveWheelbaseRadius);
        return chassisMotion;
    }

    // Input/output could be either velocity or acceleration...the math is the same.
    public WheelState solveInverseKinematics(final ChassisState chassisMotion)
    {
        WheelState wheelMotion = new WheelState();
        wheelMotion.left = (chassisMotion.linear - mEffectiveWheelbaseRadius * chassisMotion.angular) /
                mWheelRadius;
        wheelMotion.right = (chassisMotion.linear + mEffectiveWheelbaseRadius * chassisMotion.angular) /
                mWheelRadius;
        return wheelMotion;
    }

    // Solve for torques and accelerations.
    public DriveDynamics solveForwardDynamics(final ChassisState chassisVelocity, final WheelState voltage)
    {
        DriveDynamics dynamics = new DriveDynamics();
        dynamics.wheelVelocity = solveInverseKinematics(chassisVelocity);
        dynamics.chassisVelocity = chassisVelocity;
        dynamics.curvature = dynamics.chassisVelocity.angular / dynamics.chassisVelocity.linear;
        if (Double.isNaN(dynamics.curvature))
            dynamics.curvature = 0.0;
        dynamics.voltage = voltage;
        solveForwardDynamics(dynamics);
        return dynamics;
    }

    public DriveDynamics solveForwardDynamics(final WheelState wheelVelocity, final WheelState voltage)
    {
        DriveDynamics dynamics = new DriveDynamics();
        dynamics.wheelVelocity = wheelVelocity;
        dynamics.chassisVelocity = solveForwardKinematics(wheelVelocity);
        dynamics.curvature = dynamics.chassisVelocity.angular / dynamics.chassisVelocity.linear;
        if (Double.isNaN(dynamics.curvature))
            dynamics.curvature = 0.0;
        dynamics.voltage = voltage;
        solveForwardDynamics(dynamics);
        return dynamics;
    }

    // Assumptions about dynamics: velocities and voltages provided.
    public void solveForwardDynamics(DriveDynamics dynamics)
    {
        final boolean leftStationary =
                Util.epsilonEquals(dynamics.wheelVelocity.left, 0.0) && Math.abs(dynamics.voltage.left) < mLeftTransmission.frictionVoltage();
        final boolean rightStationary =
                Util.epsilonEquals(dynamics.wheelVelocity.right, 0.0) && Math.abs(dynamics.voltage.right) < mRightTransmission.frictionVoltage();
        if (leftStationary && rightStationary)
        {
            // Neither side breaks static friction, so we remain stationary.
            dynamics.wheelTorque.left = dynamics.wheelTorque.right = 0.0;
            dynamics.chassisAcceleration.linear = dynamics.chassisAcceleration.angular = 0.0;
            dynamics.wheelAcceleration.left = dynamics.wheelAcceleration.right = 0.0;
            dynamics.dcurvature = 0.0;
            return;
        }

        // Solve for motor torques generated on each side.
        dynamics.wheelTorque.left = mLeftTransmission.getTorqueForVoltage(dynamics.wheelVelocity.left, dynamics.voltage.left);
        dynamics.wheelTorque.right = mRightTransmission.getTorqueForVoltage(dynamics.wheelVelocity.right, dynamics.voltage.right);

        // Add forces and torques about the center of mass.
        dynamics.chassisAcceleration.linear = (dynamics.wheelTorque.right + dynamics.wheelTorque.left) /
                (mWheelRadius * mMass);
        // (Tr - Tl) / r_w * r_wb - drag * w = I * angular_accel
        dynamics.chassisAcceleration.angular =
                mEffectiveWheelbaseRadius * (dynamics.wheelTorque.right - dynamics.wheelTorque.left) / (mWheelRadius * mMoi)
                        - dynamics.chassisVelocity.angular * mAngularDrag / mMoi;

        // Solve for change in curvature from angular acceleration.
        // total angular accel = linear_accel * curvature + v^2 * dcurvature
        dynamics.dcurvature = (dynamics.chassisAcceleration.angular - dynamics.chassisAcceleration.linear * dynamics.curvature) /
                (dynamics.chassisVelocity.linear * dynamics.chassisVelocity.linear);
        if (Double.isNaN(dynamics.dcurvature))
            dynamics.dcurvature = 0.0;

        // Resolve chassis accelerations to each wheel.
        dynamics.wheelAcceleration.left = dynamics.chassisAcceleration.linear - dynamics.chassisAcceleration.angular * mEffectiveWheelbaseRadius;
        dynamics.wheelAcceleration.right =
                dynamics.chassisAcceleration.linear + dynamics.chassisAcceleration.angular * mEffectiveWheelbaseRadius;
    }

    // Solve for torques and voltages.
    public DriveDynamics solveInverseDynamics(final ChassisState chassisVelocity, final ChassisState chassisAcceleration)
    {
        DriveDynamics dynamics = new DriveDynamics();
        dynamics.chassisVelocity = chassisVelocity;
        dynamics.curvature = dynamics.chassisVelocity.angular / dynamics.chassisVelocity.linear;
        if (Double.isNaN(dynamics.curvature))
            dynamics.curvature = 0.0;
        dynamics.chassisAcceleration = chassisAcceleration;
        dynamics.dcurvature = (dynamics.chassisAcceleration.angular - dynamics.chassisAcceleration.linear * dynamics.curvature) /
                (dynamics.chassisVelocity.linear * dynamics.chassisVelocity.linear);
        if (Double.isNaN(dynamics.dcurvature))
            dynamics.dcurvature = 0.0;
        dynamics.wheelVelocity = solveInverseKinematics(chassisVelocity);
        dynamics.wheelAcceleration = solveInverseKinematics(chassisAcceleration);
        solveInverseDynamics(dynamics);
        return dynamics;
    }

    public DriveDynamics solveInverseDynamics(final WheelState wheelVelocity, final WheelState wheelAcceleration)
    {
        DriveDynamics dynamics = new DriveDynamics();
        dynamics.chassisVelocity = solveForwardKinematics(wheelVelocity);
        dynamics.curvature = dynamics.chassisVelocity.angular / dynamics.chassisVelocity.linear;
        if (Double.isNaN(dynamics.curvature))
            dynamics.curvature = 0.0;
        dynamics.chassisAcceleration = solveForwardKinematics(wheelAcceleration);
        dynamics.dcurvature = (dynamics.chassisAcceleration.angular - dynamics.chassisAcceleration.linear * dynamics.curvature) /
                (dynamics.chassisVelocity.linear * dynamics.chassisVelocity.linear);
        if (Double.isNaN(dynamics.dcurvature))
            dynamics.dcurvature = 0.0;
        dynamics.wheelVelocity = wheelVelocity;
        dynamics.wheelAcceleration = wheelAcceleration;
        solveInverseDynamics(dynamics);
        return dynamics;
    }

    // Assumptions about dynamics: velocities and accelerations provided, curvature and dcurvature computed.
    public void solveInverseDynamics(DriveDynamics dynamics)
    {
        // Determine the necessary torques on the left and right wheels to produce the desired wheel accelerations.
        dynamics.wheelTorque.left = mWheelRadius / 2.0 * (dynamics.chassisAcceleration.linear * mMass -
                dynamics.chassisAcceleration.angular * mMoi / mEffectiveWheelbaseRadius -
                dynamics.chassisVelocity.angular * mAngularDrag / mEffectiveWheelbaseRadius);
        dynamics.wheelTorque.right = mWheelRadius / 2.0 * (dynamics.chassisAcceleration.linear * mMass +
                dynamics.chassisAcceleration.angular * mMoi / mEffectiveWheelbaseRadius +
                dynamics.chassisVelocity.angular * mAngularDrag / mEffectiveWheelbaseRadius);

        // Solve for input voltages.
        dynamics.voltage.left = mLeftTransmission.getVoltageForTorque(dynamics.wheelVelocity.left, dynamics.wheelTorque.left);
        dynamics.voltage.right = mRightTransmission.getVoltageForTorque(dynamics.wheelVelocity.right, dynamics.wheelTorque.right);
    }

    public double getMaxAbsVelocity(double curvature, /* double dcurvature, */double maxAbsVoltage)
    {
        // Alternative implementation:
        // (Tr - Tl) * r_wb / r_w = I * v^2 * dk
        // (Tr + Tl) / r_w = 0
        // T = Tr = -Tl
        // 2T * r_wb / r_w = I*v^2*dk
        // T = 2*I*v^2*dk*r_w/r_wb
        // T = kt*(-vR/kv + V) = -kt*(-vL/vmax + V)
        // Vr = v * (1 + k*r_wb)
        // 0 = 2*I*dk*r_w/r_wb * v^2 + kt * ((1 + k*r_wb) * v / kv) - kt * V
        // solve using quadratic formula?
        // -b +/- sqrt(b^2 - 4*a*c) / (2a)

        // k = w / v
        // v = r_w*(wr + wl) / 2
        // w = r_w*(wr - wl) / (2 * r_wb)
        // Plug in max_abs_voltage for each wheel.
        final double leftSpeedAtMaxVoltage = mLeftTransmission.freeSpeedAtVoltage(maxAbsVoltage);
        final double rightSpeedAtMaxVoltage = mRightTransmission.freeSpeedAtVoltage(maxAbsVoltage);
        if (Util.epsilonEquals(curvature, 0.0))
        {
            return mWheelRadius * Math.min(leftSpeedAtMaxVoltage, rightSpeedAtMaxVoltage);
        }
        if (Double.isInfinite(curvature))
        {
            // Turn in place.  Return value meaning becomes angular velocity.
            final double wheelSpeed = Math.min(leftSpeedAtMaxVoltage, rightSpeedAtMaxVoltage);
            return Math.signum(curvature) * mWheelRadius * wheelSpeed / mEffectiveWheelbaseRadius;
        }

        final double rightSpeedIfLeftMax = leftSpeedAtMaxVoltage * (mEffectiveWheelbaseRadius * curvature +
                1.0) / (1.0 - mEffectiveWheelbaseRadius * curvature);
        if (Math.abs(rightSpeedIfLeftMax) <= rightSpeedAtMaxVoltage + Util.kEpsilon)
        {
            // Left max is active constraint.
            return mWheelRadius * (leftSpeedAtMaxVoltage + rightSpeedIfLeftMax) / 2.0;
        }
        final double leftSpeedIfRightMax = rightSpeedAtMaxVoltage * (1.0 - mEffectiveWheelbaseRadius *
                curvature) / (1.0 + mEffectiveWheelbaseRadius * curvature);
        // Right at max is active constraint.
        return mWheelRadius * (rightSpeedAtMaxVoltage + leftSpeedIfRightMax) / 2.0;
    }

    public static class MinMax
    {

        public double min;
        public double max;
    }

    // Curvature is redundant here in the case that chassis_velocity is not purely angular.  It is the responsibility of
    // the caller to ensure that curvature = angular vel / linear vel in these cases.
    public MinMax getMinMaxAcceleration(final ChassisState chassisVelocity, double curvature, /* double dcurvature, */ double maxAbsVoltage)
    {
        MinMax result = new MinMax();
        final WheelState wheelVelocities = solveInverseKinematics(chassisVelocity);
        result.min = Double.POSITIVE_INFINITY;
        result.max = Double.NEGATIVE_INFINITY;

        // Math:
        // (Tl + Tr) / r_w = m*a
        // (Tr - Tl) / r_w * r_wb - drag*w = i*(a * k + v^2 * dk)

        // 2 equations, 2 unknowns.
        // Solve for a and (Tl|Tr)

        final double linearTerm = Double.isInfinite(curvature) ? 0.0 : mMass * mEffectiveWheelbaseRadius;
        final double angularTerm = Double.isInfinite(curvature) ? mMoi : mMoi * curvature;

        final double dragTorque = chassisVelocity.angular * mAngularDrag;

        // Check all four cases and record the min and max valid accelerations.
        for (boolean left : Arrays.asList(false, true))
        {
            for (double sign : Arrays.asList(1.0, -1.0))
            {
                final DCMotorTransmission fixedTransmission = left ? mLeftTransmission : mRightTransmission;
                final DCMotorTransmission variableTransmission = left ? mRightTransmission : mLeftTransmission;
                final double fixedTorque = fixedTransmission.getTorqueForVoltage(wheelVelocities.get(left), sign *
                        maxAbsVoltage);
                double variableTorque = 0.0;
                // NOTE: variable_torque is wrong.  Units don't work out correctly.  We made a math error somewhere...
                // Leaving this "as is" for code release so as not to be disingenuous, but this whole function needs
                // revisiting in the future...
                if (left)
                {
                    variableTorque =
                            ((/*-moi_ * chassis_velocity.linear * chassis_velocity.linear * dcurvature*/ -dragTorque) * mMass * mWheelRadius
                                    + fixedTorque *
                                            (linearTerm + angularTerm))
                                    / (linearTerm - angularTerm);
                }
                else
                {
                    variableTorque =
                            ((/* moi_ * chassis_velocity.linear * chassis_velocity.linear * dcurvature */ +dragTorque) * mMass * mWheelRadius
                                    + fixedTorque *
                                            (linearTerm - angularTerm))
                                    / (linearTerm + angularTerm);
                }
                final double variableVoltage = variableTransmission.getVoltageForTorque(wheelVelocities.get(!left), variableTorque);
                if (Math.abs(variableVoltage) <= maxAbsVoltage + Util.kEpsilon)
                {
                    double accel = 0.0;
                    if (Double.isInfinite(curvature))
                    {
                        accel = (left ? -1.0 : 1.0) * (fixedTorque - variableTorque) * mEffectiveWheelbaseRadius
                                / (mMoi * mWheelRadius) - dragTorque / mMoi /*- chassis_velocity.linear * chassis_velocity.linear * dcurvature*/;
                    }
                    else
                    {
                        accel = (fixedTorque + variableTorque) / (mMass * mWheelRadius);
                    }
                    result.min = Math.min(result.min, accel);
                    result.max = Math.max(result.max, accel);
                }
            }
        }
        return result;
    }

    // Can refer to velocity or acceleration depending on context.
    public static class ChassisState
    {

        /** Either meters/sec or meters/sec^2 */
        public double linear;
        /** Either radians/sec or radians/sec^2 */
        public double angular;

        public ChassisState(double linear, double angular)
        {
            this.linear = linear;
            this.angular = angular;
        }

        public ChassisState()
        {
        }

        @Override
        public String toString()
        {
            DecimalFormat fmt = new DecimalFormat("#0.000");
            return fmt.format(linear) + ", " + fmt.format(angular);
        }
    }

    // Can refer to velocity, acceleration, torque, voltage, etc., depending on context.
    public static class WheelState
    {

        /** Either radians/sec, radians/sec^2, Newton meters, or Volts */
        public double left, right;

        public WheelState(double left, double right)
        {
            this.left = left;
            this.right = right;
        }

        public WheelState()
        {
        }

        public double get(boolean getLeft)
        {
            return getLeft ? left : right;
        }

        public void set(boolean setLeft, double val)
        {
            if (setLeft)
            {
                left = val;
            }
            else
            {
                right = val;
            }
        }

        @Override
        public String toString()
        {
            DecimalFormat fmt = new DecimalFormat("#0.000");
            return fmt.format(left) + ", " + fmt.format(right);
        }
    }

    // Full state dynamics of the drivetrain.
    public static class DriveDynamics
    {

        /** 1/meters */
        public double curvature = 0.0;
        /** 1/meters^2 */
        public double dcurvature = 0.0;
        public ChassisState chassisVelocity = new ChassisState(); // m/s
        public ChassisState chassisAcceleration = new ChassisState(); // m/s^2
        public WheelState wheelVelocity = new WheelState(); // rad/s
        public WheelState wheelAcceleration = new WheelState(); // rad/s^2
        public WheelState voltage = new WheelState(); // V
        public WheelState wheelTorque = new WheelState(); // N m
    }
}
