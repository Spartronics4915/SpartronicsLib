package com.spartronics4915.lib.math.threedim.math3;

import java.util.ArrayList;

/* Affine3 is a specialization of DMatrix4x4.  Its purpose is to
 * present a constrained interface to users, focused on the expression
 * and manipulation of coordinate frames in the context of FRC robotics.
 * Strictly speaking we require a 3x4 matrix to capture affine transforms.
 * It's common in CGI applications to use a 4x4 because it can also be
 * use to represent projective transformations as occur in camera lenses.
 * 
 * There are many learning resources for affine transformations.
 * Here are a few:
 *  - http://graphics.cs.cmu.edu/nsp/course/15-462/Spring04/slides/04-transform.pdf
 *  - Matrices and transformations. Ronald Goldman.
 *     In "Graphics Gems I", pp 472-475. Morgan Kaufmann, 1990.
 *  - More matrices and transformations: shear and pseudo-perspective.
 *    Ronald Goldman. In "Graphics Gems II", pp 320-323. Morgan Kaufmann, 1991.
 *  - Decomposing a matrix into simple transformations. Spencer Thomas.
 *     In "Graphics Gems II", pp 320-323. Morgan Kaufmann, 1991.
 * 
 *  Basic idea: an affine matrix can be used to describe coordinate-system
 *  conversions as we require when expressing how to convert from the
 *  camera coordinate-frame to the robot-coordinate frame.  Here the camera
 *  has a natural coordinate system and so does the robot.  The question
 *  we'd like to answer: what does a point (or direction) in the camera's 
 *  coordinate system mean to the robot?  Once we've "designed" a transformation 
 *  matrix, we can "multiply" (via "dot product") the point in one coordinate 
 *  system by the transformation matrix to obtain the point in another 
 *  coordinate system. But wait, there's more! We can "chain" coordinate 
 *  systems together to convert points from camera, then to robot, then to 
 *  field.  A combined matrix can be produced (by concatenate) to represent
 *  multiple transformations in one matrix.  We can also "invert" these 
 *  transformations and now we can compute where a field location should 
 *  appear in the camera coordinate system.
 *
 *  To construct/design a matrix we must carefully consider the way that
 *  we must rotate one coordinate system to obtain another.  There are
 *  a number of ways to characterize this rotation, some more intuitive
 *  than others.  If you are dealing with 90 degree, rigid transformations,
 *  the "fromAxes" approach may be best.  Euler angles may be the most
 *  traditional in robotics, but multiple, sequential rotations can be difficult 
 *  to grasp.  Quaternions are very useful for interpolating arbitrary 3D
 *  rotations (and avoid "gimbal lock") but harder to design from scratch.  
 *  Quaternions are also a robust and compact representation for rotations.  
 *  So we offer/support a range of rotation specification methodologies.  
 *  See also the sibling Quaternion class.
 * 
 * This module follows the "column vectors on the right" and "row major storage"
 * (C contiguous) conventions. The translation components are in the right column
 * of the transformation matrix, i.e. this.a14-a34.
 * The transpose of the Affine3 matrices may have to be used to interface
 * with other graphics systems, e.g. OpenGL's glMultMatrixd().
 */

public class Affine3
{
    /* static convenience methods and objects --------------------------------*/
    public final static Affine3 Identity = new Affine3();

    public static Affine3 fromRotation(double angle, Vec3 axis)
    {
        return new Affine3(angle, axis, null);
    }

    public static Affine3 fromRotation(double angle, Vec3 axis, Vec3 pivot)
    {
        return new Affine3(angle, axis, pivot);
    }

    public static Affine3 fromAxes(Vec3 x, Vec3 y, Vec3 z)
    {
        return new Affine3(x, y, z);
    }

    public static Affine3 fromTranslation(Vec3 xlate)
    {
        return new Affine3(xlate);
    }

    public static Affine3 fromTranslation(double x, double y, double z)
    {
        return new Affine3(x, y, z);
    }

    public static Affine3 fromQuaternion(Quaternion q)
    {
        return new Affine3(q); 
    } 

    public static Affine3 concatenate(Affine3 ... alist)
    {
        Affine3 result = new Affine3();
        for(Affine3 a : alist)
            result.multiply(a);
        return result;
    }

    /* --------------------------------------------------------------------*/
    private Matrix4 mMatrix;

    public Affine3()
    {
        this.mMatrix = Matrix4.identity();
    }

    public Affine3(Matrix4 m)
    {
        this.mMatrix = m;
    }

    public Affine3(double [][]d)
    {
        this.mMatrix = new Matrix4(d);
    }

    /**
     * Construct an affine matrix as a translation
     * @param xlate - A Vec3 expressing the translation amount.
     */
    public Affine3(Vec3 xlate)
    {
        this(xlate.getX(), xlate.getY(), xlate.getZ());
    }

    /**
     * Contruct an affine matrix as a translation
     * @param x
     * @param y
     * @param z
     */
    public Affine3(double x, double y, double z)
    {
        this.mMatrix = Matrix4.identity();
        double d[][] = this.mMatrix.getDataRef();
        d[0][3] = x;
        d[1][3] = y;
        d[2][3] = z;
    }

    /**
     * Construct Affine3 representing rotation of axes to targets. NB:
     * all targets should be unit vectors.
     * @param xtgt
     * @param ytgt
     * @param ztgt
     */
    public Affine3(final Vec3 xtgt, final Vec3 ytgt, final Vec3 ztgt)
    {
        this.mMatrix = Matrix4.identity();
        double d[][] = this.mMatrix.getDataRef();
        d[0][0] = xtgt.getX();
        d[1][0] = xtgt.getY();
        d[2][0] = xtgt.getZ();
        d[0][1] = ytgt.getX();
        d[1][1] = ytgt.getY();
        d[2][1] = ytgt.getZ();
        d[0][2] = ztgt.getX();
        d[1][2] = ztgt.getY();
        d[2][2] = ztgt.getZ();
    }

    /**
     * Construct an Affine3 given a concise string represention of the form: 
     *     "o x y z q 1 2 3 4",
     * NB: this rep doesn't allow for scale and skew.  For now this is fine.
     * 
     * @param str
     * @return Affine3 transformation matrix
     */
    public Affine3(String str)
    {
        String[] vals = str.split(" ");
        assert vals.length == 9;
        assert vals[0].equals("o");
        assert vals[4].equals("q");

        double x = Double.parseDouble(vals[1]);
        double y = Double.parseDouble(vals[2]);
        double z = Double.parseDouble(vals[3]);
        double q0 = Double.parseDouble(vals[5]);
        double q1 = Double.parseDouble(vals[6]);
        double q2 = Double.parseDouble(vals[7]);
        double q3 = Double.parseDouble(vals[8]);

        Affine3 trans = new Affine3(x,y,z);
        Affine3 rot = new Affine3(new Quaternion(q0, q1, q2, q3));
        Affine3 result = Affine3.concatenate(trans, rot);
        this.mMatrix = result.mMatrix;
    } 

    /**
     * Construct an Affine3 given a Quaternion.
     * @param q - A quaternion representation of rotation.
     */
    public Affine3(Quaternion q)
    {
        this.mMatrix = Matrix4.identity();
        Matrix3 m = q.asMatrix3();
        this.mMatrix.setSubMatrix(m.getDataRef(), 0, 0);
    }

    /**
     * construct an Affine3 from a simple rotation around an optional point.
     * @param angle - amount to rotate in degrees
     * @param axis - unit vector rep of axis, ie: Vec3(0,1,0) is '+y' 
     * @param pt - optional rotation pivot, null implies Vec3(0,0,0)
     * @return rotation matrix
     */
    public Affine3(double angle, final Vec3 axis, final Vec3 pt)
    {
        Quaternion q = new Quaternion(angle, axis);
        this.mMatrix = new Matrix4(q.asMatrix3());
        if(pt != null)
        {
            Vec3 pp = pt.subtract(this.transformVector(pt));
            double[][] d4 = mMatrix.getDataRef();
            d4[0][3] = pp.getX();
            d4[1][3] = pp.getY();
            d4[2][3] = pp.getZ();
        }
    }

    /**
     * Expose our matrix represention to the outer world.
     * @return ejml DMatrix4x4 
     */
    public Matrix4 asMatrix()
    {
        return this.mMatrix;
    }

    /**
     * Convert the matrix to a compact string representation suitable for
     * atomic transmission via networkt tables.
     * @return compact string representation
     */
    public String asString()
    {
        return null;
    }

    public boolean equals(final Affine3 rhs)
    {
        return this.equals(rhs, 1e-9);
    }

    public boolean equals(final Affine3 rhs, double epsilon)
    {
        return this.mMatrix.equals(rhs.mMatrix, epsilon);
    }

    /**
     * @return the sum of diagonal elements of matrix
     */
    public double trace()
    {
        return this.mMatrix.getTrace();
    }

    public void print()
    {
        this.mMatrix.print();
    }

    /**
     * Apply a rotation to this instance
     * @param angle - measured in degrees
     * @param dir - unit vector representing the rotational pole
     * @return this (for chaining)
     */
    public Affine3 rotate(double angle, Vec3 dir)
    {
        Affine3 rot = Affine3.fromRotation(angle, dir);
        this.multiply(rot);
        return this;
    }

    /**
     * Apply a translation to this instance. Take care with order
     * of operations between rotations and translations.
     * @param v - vector representing translation
     * @return this (for chaining)
     */
    public Affine3 translate(Vec3 v)
    {
        return this.translate(v.getX(), v.getY(), v.getZ());
    }

    public Affine3 translate(double x, double y, double z)
    {
        Affine3 xlate = Affine3.fromTranslation(x, y, z);
        this.multiply(xlate);
        return this;
    }

    public Vec3 getTranslation()
    {
        double m4[][] = this.mMatrix.getDataRef();
        return new Vec3(m4[0][3], m4[1][3], m4[2][3]);
    }
    
    /**
     * multiplies this Affine3 by another, this now contains product.
     */
    public void multiply(final Affine3 rhs)
    {
        Affine3 prod = this.asProduct(rhs);
        this.mMatrix = prod.mMatrix;
    }

    public Affine3 asProduct(final Affine3 rhs)
    {
        return new Affine3(this.mMatrix.multiply(rhs.mMatrix).getDataRef());
    }

    /**
     * Compute matrix inverse
     * @return the inverse of this Affine3
     */
    public void invert()
    {
        Matrix4 imat = this.mMatrix.invert();
        this.mMatrix = imat;
    }

    public Affine3 asInverse()
    {
        return new Affine3(this.mMatrix.invert());
    }

    public Vec3 transformPoint(final Vec3 in)
    {
        double[] vec4 = new double[4];
        vec4[0] = in.getX();
        vec4[1] = in.getY();
        vec4[2] = in.getZ();
        vec4[3] = 1; // <- point
        double opt[] = this.mMatrix.operate(vec4);
        return new Vec3(opt[0], opt[1], opt[2]);
        // return new Vec3(out4.a1, out4.a2, out4.a3);
    }

    public Vec3 transformVector(final Vec3 in)
    {
        double[] vec4 = new double[4];
        vec4[0] = in.getX();
        vec4[1] = in.getY();
        vec4[2] = in.getZ();
        vec4[3] = 0; // <- vector
        double opt[] = this.mMatrix.operate(vec4);
        return new Vec3(opt[0], opt[1], opt[2]);
    }

    public ArrayList<Vec3> transformPoints(final Vec3 ...in)
    {
        ArrayList<Vec3> result = new ArrayList<Vec3>();
        double[] vec4 = new double[4];
        vec4[3] = 1; // <---  as point
        for(final Vec3 v : in)
        {
            vec4[0] = v.getX();
            vec4[1] = v.getY();
            vec4[2] = v.getZ();
            double opt[] = this.mMatrix.operate(vec4);
            result.add(new Vec3(opt[0], opt[1], opt[2]));
        }
        return result;
    }

    public ArrayList<Vec3> transformVectors(final Vec3 ...in)
    {
        ArrayList<Vec3> result = new ArrayList<Vec3>();
        double[] vec4 = new double[4];
        vec4[3] = 0; // <---  as vector
        for(final Vec3 v : in)
        {
            vec4[0] = v.getX();
            vec4[1] = v.getY();
            vec4[2] = v.getZ();
            double opt[] = this.mMatrix.operate(vec4);
            result.add(new Vec3(opt[0], opt[1], opt[2]));
        }
        return result;
    }

    public ArrayList<Vec3> transformBases()
    {
        return this.transformVectors(new Vec3(1,0,0), 
                                     new Vec3(0,1,0),
                                     new Vec3(0,0,1));
    }
}