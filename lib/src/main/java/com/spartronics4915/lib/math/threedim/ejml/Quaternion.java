package com.spartronics4915.lib.math.threedim.ejml;

import org.ejml.data.DMatrix4x4;
import org.ejml.data.DMatrix4;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.fixed.CommonOps_DDF4;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.EigenDecomposition;
import org.ejml.dense.row.decomposition.eig.SymmetricQRAlgorithmDecomposition_DDRM;
import org.ejml.data.Complex_F64;

/*
 * bizarre usage of ejml:
 * 
 * http://ejml.org/javadoc/org/ejml/dense/fixed/CommonOps_DDF4.html
 * https://ejml.org/wiki/index.php?title=Matlab_to_EJML
 */

/**
 *  Robust+compact representation of 3D rotation
 *
 *  Quaternions w+ix+jy+kz are represented as [w, x, y, z].
 *  see: 
 *      ttps://eater.net/quaternions/intro
 *      https://www.youtube.com/watch?v=jlskQDR8-bY  (Mathoma)
 *      https://www.youtube.com/watch?v=d4EgbgTm0Bg (3Blue1Brown)
 *      https://eater.net/quaternions (Ben Eater)
 */

public class Quaternion
{
    static final double kEpsilon = 1e-9;

    /* factory static methods -----------------------------------------------*/

    /* methods -----------------------------------------------*/
    private DMatrix4 mQ;

    /**
     * 4 double contructor - each of the 4 quaternion components
     * @param q0
     * @param q1
     * @param q2
     * @param q3
     */
    public Quaternion(double q0, double q1, double q2, double q3)
    {
        this.mQ = new DMatrix4();
        this.mQ.a1 = q0;
        this.mQ.a2 = q1;
        this.mQ.a3 = q2;
        this.mQ.a4 = q3;
    }

    /**
     * default contructor, produces identity
     */
    public Quaternion()
    {
        this(1, 0, 0, 0);
    }

    /**
     * angle/axis contructor
     * @param angle - measured in degrees
     * @param axis
     */
    public Quaternion(double angle, Vec3 axis)
    {
        this.mQ = new DMatrix4();
        this.mQ.a1 = 0.0;
        this.mQ.a2 = axis.a1;
        this.mQ.a3 = axis.a2;
        this.mQ.a4 = axis.a3;
        double len = axis.length();
        double rads = Math.toRadians(angle);
        if (len > kEpsilon)
        {
            double s = Math.sin(rads / 2.0) / len;
            CommonOps_DDF4.scale(s, this.mQ);
        }
        this.mQ.a1 = Math.cos(rads / 2.0);
    }

    /**
     * euler-angle constructor
     * @param ai
     * @param aj
     * @param ak
     * @param axes
     */
    public Quaternion(double ai, double aj, double ak, String axes)
    {
        assert (false); // XXX: implement me if need-be
    }

    /**
     * 
     * @param a - an Affine3 to decompose
     * @param precise - if true, input is assumed to be a precise rotation 
     *   matrix and a faster algorithm is used. If in doubt, set to false.
     */
    public Quaternion(Affine3 a)
    {
        this(a, false); // not the precise/fast method...
    }

    public Quaternion(Affine3 a, boolean precise)
    {
        // quaternion_from_matrix
        DMatrix4x4 m = a.asMatrix();
        this.mQ = new DMatrix4();
        if (precise)
        {
            // If precise is True, the input matrix is assumed to be a precise
            // rotation matrix and a faster algorithm is used.
            double t = a.trace();
            if (t > m.a44)
            {
                this.mQ.a1 = t;
                this.mQ.a2 = m.a21 - m.a12;
                this.mQ.a3 = m.a13 - m.a31;
                this.mQ.a4 = m.a32 - m.a23;
            }
            else
            {
                int i = 0, j = 1, k = 2;
                if (m.a22 > m.a11)
                {
                    i = 1;
                    j = 2;
                    k = 0;
                }
                // m.get has index origin of 0 !!
                if (m.a33 > m.get(i, i))
                {
                    i = 2;
                    j = 0;
                    k = 1;
                }
                t = m.get(i, i) - (m.get(j, j) + m.get(k, k)) + m.a44;
                double[] q = {0, 0, 0, 0};
                q[i] = t;
                q[j] = m.get(i, j) + m.get(j, i);
                q[k] = m.get(k, i) + m.get(i, k);
                q[3] = m.get(k, j) - m.get(j, k);
                this.mQ.a1 = q[3];
                this.mQ.a2 = q[0];
                this.mQ.a3 = q[1];
                this.mQ.a4 = q[2];
            }
            double rescale = .5 / Math.sqrt(t * m.a44);
            CommonOps_DDF4.scale(rescale, this.mQ);
        }
        else
        {
            double[] w = {0, 0, 0, 0}; // same w as numpy.eigh
            DMatrix4x4 k = new DMatrix4x4();
            k.a11 = m.a11 - m.a22 - m.a33;
            k.a12 = 0;
            k.a13 = 0;
            k.a14 = 0;
            k.a21 = m.a12 + m.a21;
            k.a22 = m.a22 - m.a11 - m.a33;
            k.a23 = 0;
            k.a24 = 0;
            k.a31 = m.a13 + m.a31;
            k.a32 = m.a23 + m.a32;
            k.a33 = m.a33 - m.a11 - m.a22;
            k.a34 = 0;
            k.a41 = m.a32 - m.a23;
            k.a42 = m.a13 - m.a31;
            k.a43 = m.a21 - m.a12;
            k.a44 = m.a11 + m.a22 + m.a33;
            CommonOps_DDF4.scale(1. / 3, k);
            // This shorthand represents 0 as placeholders for the symmetric
            // components. numpy.linalg.eigh defaults to using the lower
            // diagonal, while it appears the ejml uses the entire matrix;
            k.a12 = k.a21;
            k.a13 = k.a31;
            k.a14 = k.a41;
            k.a23 = k.a32;
            k.a24 = k.a42;
            k.a34 = k.a43;
            // k.print();
            DMatrixRMaj rm2 = new DMatrixRMaj(k);
            EigenDecomposition<DMatrixRMaj> eig = DecompositionFactory_DDRM.eig(true, true);
            // EigenDecomposition<DMatrixRMaj> eig = DecompositionFactory_DDRM.eig(4, true);
            eig.decompose(rm2);
            // find eigenvector associated with largest eigenvalue
            if (eig instanceof SymmetricQRAlgorithmDecomposition_DDRM)
            {
                DMatrixRMaj[] VT = {null, null, null, null};
                int argMax = -1;
                double magMax2 = 0;
                for (int i = 0; i < eig.getNumberOfEigenvalues(); i++)
                {
                    Complex_F64 v = ((SymmetricQRAlgorithmDecomposition_DDRM) eig).getEigenvalue(i);
                    assert (v.isReal());
                    w[i] = v.getReal();
                    VT[i] = eig.getEigenVector(i);
                    double mag2 = v.getMagnitude2();
                    if (mag2 > magMax2)
                    {
                        magMax2 = mag2;
                        argMax = i;
                    }
                }
                assert (argMax > -1);

                // re-arrange
                this.mQ.a1 = VT[argMax].get(3, 0);
                this.mQ.a2 = VT[argMax].get(0, 0);
                this.mQ.a3 = VT[argMax].get(1, 0);
                this.mQ.a4 = VT[argMax].get(2, 0);

                if (this.mQ.a1 < 0)
                {
                    this.mQ.a1 *= -1;
                    this.mQ.a2 *= -1;
                    this.mQ.a3 *= -1;
                    this.mQ.a4 *= -1;
                }
            }
            else
                assert (false);
        }
    }

    public DMatrix4 asDMatrix4()
    {
        return this.mQ;
    }

    /**
     * converts the current quaternion values into 4x4 matrix form.
     * nb: translation and projection fields are 0.
     * @return
     */
    public DMatrix4x4 asDMatrix4x4()
    {
        DMatrix4x4 result = new DMatrix4x4();
        DMatrix4 q = this.mQ;
        double n = CommonOps_DDF4.dot(q, q);
        if (n < kEpsilon)
            CommonOps_DDF4.setIdentity(result);
        else
        {
            CommonOps_DDF4.scale(Math.sqrt(2.0 / n), q);
            /* outer product(q,q) */
            DMatrix4x4 qq = new DMatrix4x4();
            qq.a11 = q.a1 * q.a1;
            qq.a12 = q.a2 * q.a1;
            qq.a13 = q.a3 * q.a1;
            qq.a14 = q.a4 * q.a1;
            qq.a21 = q.a1 * q.a2;
            qq.a22 = q.a2 * q.a2;
            qq.a23 = q.a3 * q.a2;
            qq.a24 = q.a4 * q.a2;
            qq.a31 = q.a1 * q.a3;
            qq.a32 = q.a2 * q.a3;
            qq.a33 = q.a3 * q.a3;
            qq.a34 = q.a4 * q.a3;
            qq.a41 = q.a1 * q.a4;
            qq.a42 = q.a2 * q.a4;
            qq.a43 = q.a3 * q.a4;
            qq.a44 = q.a4 * q.a4;

            result.a11 = 1. - qq.a33 - qq.a44;
            result.a12 = qq.a23 - qq.a41;
            result.a13 = qq.a24 + qq.a31;
            result.a14 = 0;

            result.a21 = qq.a23 + qq.a41;
            result.a22 = 1. - qq.a22 - qq.a44;
            result.a23 = qq.a34 - qq.a21;
            result.a24 = 0.;

            result.a31 = qq.a24 - qq.a31;
            result.a32 = qq.a34 + qq.a21;
            result.a33 = 1. - qq.a22 - qq.a33;
            result.a34 = 0.;

            result.a41 = 0;
            result.a42 = 0;
            result.a43 = 0;
            result.a44 = 1;
        }
        return result;
    }

    public boolean equals(Quaternion rhs)
    {
        return this.equals(rhs, 1e-9);
    }

    public boolean equals(Quaternion rhs, double epsilon)
    {
        if (Math.abs(this.mQ.a1 - rhs.mQ.a1) > epsilon)
            return false;
        if (Math.abs(this.mQ.a2 - rhs.mQ.a2) > epsilon)
            return false;
        if (Math.abs(this.mQ.a3 - rhs.mQ.a3) > epsilon)
            return false;
        if (Math.abs(this.mQ.a4 - rhs.mQ.a4) > epsilon)
            return false;
        return true;
    }

    public String asString()
    {
        return String.format("q %g %g %g %g", this.mQ.a1, this.mQ.a2, this.mQ.a3, this.mQ.a4);

    }
}
