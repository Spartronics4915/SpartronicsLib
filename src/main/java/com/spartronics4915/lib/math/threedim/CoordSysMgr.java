package com.spartronics4915.lib.math.threedim;

import com.spartronics4915.lib.math.threedim.math3.*;
import edu.wpi.first.wpilibj.geometry.*;
import com.spartronics4915.lib.util.Units;

/**
 * CoordSysMgr captures the coordinate-system chain (kinematics) associated
 * with a camera mounted on a potentially-moving mount attached to a robot.
 * To produce season-specific behavior, please subclass this class.
 */

/*
 * Robot field pose (ie: robot position/heading in field coordinates)
 * aka the robotToField 
 *
 * field coords (z is up)
 * field is approx x: [0, 52*12], y: [-26*12, 26*12] (z is up), in theory we 
 * don't need to know its exact coords.
 * 
 * Field
 *                    y (+) (-312,312)
 *                    |_____ ______. 
 *     Blue Alliance  |            |  Red Alliance
 *                    o-----x      | (0,640)
 *                    |         R  |
 *                    |____________| 
 *
 * Robot
 *                       y
 *                 ._____|____. 
 *                 |     |    |             
 *                 |     o-----x
 *              x--- M        |
 *                 |_|________| 
 *                   y
 * 
 * Mount                    |     Camera (axometric view) - camera plane is xy
 *                .___.     |
 *                | C |     |          y 
 *                |   |     |          |
 *                | z |     |          |
 *                | | |     |          C ------ -z
 *         x -----|-M |     |         /
 *                |___|     |        x
 */

public class CoordSysMgr
{
    private Affine3 mCamToMount;
    private Affine3 mMountToRobot;
    private Affine3 mFieldToMount;
    private Affine3 mCamToRobot;
    private Affine3 mRobotToField;
    private Affine3 mCamToField;
    private boolean mDirty;

    public CoordSysMgr()
    {
        mCamToMount = new Affine3();
        mMountToRobot = new Affine3();
        mCamToRobot = new Affine3();
        mRobotToField = new Affine3();
        mCamToField = new Affine3();
        mFieldToMount = new Affine3();
        mDirty = true;
    }

    /**
     * Sets the *fixed* mounting of the camera on the mount or robot.
     * We expect this method to be called upon robot's subsystem
     * initialization (and stored in a Constants file).
     * @param camPose- a string representing the pose obtained by
     *   manually constructing an Affine3, then invoking its asString() method.
     *   The string representation is compact and can be atomically transmitted
     *   via network tables.
     */
    public void setCamToMount(String camPose)
    {
        this.mCamToMount = new Affine3(camPose);
    }

    public void setCamToMount(final Affine3 cToM)
    {
        this.mCamToMount = cToM;
    }

    /**
     * Sets the mounting of the mount onto the robot.  If
     * the camera is mounted directly to the robot this is unneeded.
     * Otherwise we expect thie method to be called upon robot's subsystem
     * intialization (and stored in a Constants file) and potentially 
     * periodically during the match.
     * @param mountPose - a string representing the pose obtained by
     *   manually constructing an Affine3, then invoking its asString() method.
     *   The string representation is compact and can be atomically transmitted
     *   via network tables.
     */
    public void setMountToRobot(String mountPose)
    {
        this.setMountToRobot(new Affine3(mountPose));
    }

    /**
     * If the vision camera is mounted on an articulated mechanism (eg turret)
     * we expect to receive an update to the mount description.  The format
     * of mountPose may differ from season-to-season.  In the case of a turret
     * we expect that the mountPose is merely an angle to capture the current
     * turret angle.  Note that this must be composed with other mount 
     * characteristics (ie: its offset and orientation of the zero angle)
     */
    public void setMountToRobot(Affine3 mToR)
    {
        this.mMountToRobot = mToR;
        this.mDirty = true;
    }

    /**
     * Updates internal state via a standard Pose2d (assumed in meters).
     * @param fieldToVehicle - usually obtained via 
     *   RobotStateMap.getLatestFieldToVehicle.
     */
    public void updateRobotPose(Pose2d fieldToVehicle)
    {
        Translation2d td = fieldToVehicle.getTranslation();
        double x = Units.metersToInches(td.getX());
        double y = Units.metersToInches(td.getY());
        double angle = fieldToVehicle.getRotation().getDegrees();

        this.updateRobotPose(x, y, angle);
    }

    /**
     * Updates robot pose via a string.
     * @param rposeString "x y angle" x, y in inches, angle in degrees
     */
    public void updateRobotPose(String rposeString)
    {
        String[] vals = rposeString.split(" ");
        assert vals.length == 3;
        double x = Double.parseDouble(vals[0]);
        double y = Double.parseDouble(vals[1]);
        double angle = Double.parseDouble(vals[2]);
        this.updateRobotPose(x, y, angle);
    }

    /**
     * Updates internal notion of robot pose via numbers.
     * @param x - x coordinate of robot origin on field in inches
     * @param y - y coordinate of robot origin on field in inches
     * @param angle - heading of robot in degrees.  If x of robot points to 
     * x of field, the angle is zero.
     */
    public void updateRobotPose(double x, double y, double angle)
    {
        Affine3 xlate =  Affine3.fromTranslation(x, y, 0);
        Affine3 rot = Affine3.fromRotation(angle, new Vec3(0,0,1));
        this.mRobotToField = Affine3.concatenate(xlate, rot);
        this.mDirty = true;
    }

    /**
     * Given a known robot heading, a measured target in robot coords
     * and the known field coordinates of the same target, compute the
     * full robotToField coordinates.
     * @param knownHeading - measured in degrees in field coordinates
     * @param robotTgt - target pt in robot coordinates (from vision) -
     *   measured in inches.
     * @param fieldTgt - known target location in field coords - measured
     *   in inches.
     */
    public void updateRobotPose(double knownHeading, final Vec3 robotTgt, 
                                final Vec3 fieldTgt)
    {
        Affine3 rot = Affine3.fromRotation(knownHeading, Vec3.ZAxis);
        // transform robotTgtPt to field by applying xy rotation
        Vec3 rotTgt = rot.transformPoint(robotTgt);
        // to move robot origin to correct field location:
        //  xf = x + dx
        Vec3 dx = fieldTgt.subtract(rotTgt);
        Affine3 xlate = Affine3.fromTranslation(dx);
        this.mRobotToField = Affine3.concatenate(xlate, rot);
        this.mDirty = true;
    }

    /**
     * This is the primary method to convert a camera-space point (a vision target)
     * into field coordinates suitable for consumption by the RobotStateEsimator.
     * @param pt - coordinates of target in camera space.  A point at the center
     * of the camera frame 10 feet away would be (0, 0, -120).
     * @return the target point converted to field coordinates.
     */
    public Vec3 camPointToField(Vec3 pt)
    {
        this.rebuildTransforms();
        return this.mCamToField.transformPoint(pt);
    }

    /**
     * This is the primary method to convert a camera-space direction into
     * into field coordinates. May be useful to ensure that the transform
     * chain has the expected result.
     * @param dir - unit-length direction vector in camera space.
     * @return the camera direction represented in field coordinates.
     */
    public Vec3 camDirToField(Vec3 dir)
    {
        this.rebuildTransforms();
        return this.mCamToField.transformVector(dir);
    }

    /**
     * This is the primary method to convert a camera-space point (a vision target)
     * into robot coordinates. Additional work may be needed to infer robot
     * position.
     * @param pt - coordinates of target in camera space.  A point at the center
     * of the camera frame 10 feet away would be (0, 0, -120).
     * @return the target point converted to robot-relative coordinates.
     */
    public Vec3 camPointToRobot(Vec3 pt)
    {
        this.rebuildTransforms();
        return this.mCamToRobot.transformPoint(pt);

    }

    /**
     * This is the primary method to convert a camera-space direction into
     * into robot coordinates. Additional work may be need infer robot
     * orientation on the field.
     * @param dir - unit-length direction vector in camera space.
     * @return the camera direction represented in robot-relative coordinates.
     */
    public Vec3 camDirToRobot(Vec3 dir)
    {
        this.rebuildTransforms();
        return this.mCamToRobot.transformVector(dir);
    }

    /**
     * Convert a target in camera space into turret space.  This
     * transformation can be used to compute a shooting distance as
     * follows:  
     *    Vec3 dist = camPointToMount(camTargetPt);
     *    dist.a2 = 0; // if we don't care about height
     *    double distance = dist.length();
     */
    public Vec3 camPointToMount(Vec3 pt)
    {
        this.rebuildTransforms();
        return this.mCamToMount.transformVector(pt);
    }

    /**
     * Convert a robot-space point into field coordinates.
     * @param pt - a point in the robot's coordinate system
     * @return - a point in the field's coordinate system
     */
    public Vec3 robotPointToField(Vec3 pt)
    {
        // no rebuild needed since there's no chain involved
        return this.mRobotToField.transformPoint(pt);
    }

    /**
     * Convert a robot-space direction into field coordinates.
     * @param dir - a direction in the robot's coordinate system
     * @return - a direction in the field's coordinate system
     */
    public Vec3 robotDirToField(Vec3 dir)
    {
        // no rebuild needed since there's no chain involved
        return this.mRobotToField.transformVector(dir);
    }

    public Vec3 fieldDirToMount(Vec3 dir)
    {
        this.rebuildTransforms();
        return this.mFieldToMount.transformVector(dir);
    }

    public Vec3 fieldPointToMount(Vec3 pt)
    {
        this.rebuildTransforms();
        return this.mFieldToMount.transformPoint(pt);
    }

    /**
     * updates the transform chain whenever any inputs have changed.
     */
    private void rebuildTransforms()
    {
        if(this.mDirty)
        {
            this.mCamToRobot = Affine3.concatenate(this.mMountToRobot,
                                                this.mCamToMount);
            this.mCamToField = Affine3.concatenate(this.mRobotToField, 
                                                this.mCamToRobot);
            this.mFieldToMount = Affine3.concatenate(this.mRobotToField,
                                                this.mMountToRobot).asInverse();
            this.mDirty = false;
        }
    }

}