package com.spartronics4915.lib.math.threedim;

import com.spartronics4915.lib.math.threedim.math3.*;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;

import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;


class Math3Tests
{
    static final double kEpsilon = 1e-9;

    @Test
    public void testMatrix()
    {
        Matrix4 m0 = Matrix4.identity();
        double p0[] = {1, 2, 3, 1};
        double v0[] = {1, 2, 3, 0};
        double op0[] = m0.operate(p0);
        double ov0[] = m0.operate(v0);

        double data[][] = m0.getDataRef();

        /* build a trivial scale + translate */
        data[0][0] = 2;
        data[1][1] = 3;
        data[2][2] = 4;

        // translate X, Y, X, in "opencv rep"
        data[0][3] = .35; 
        data[1][3] = .39;
        data[2][3] = .43;

        /* for opencv rep, we want operate, not premultiply */
        op0 = m0.operate(p0);
        ov0 = m0.operate(v0);
        assertEquals(op0[0], 2.35, kEpsilon);
        assertEquals(op0[3], 1, kEpsilon);

        
        /* for opengl rep, we'd want premultiply, not operate */
        op0 = m0.preMultiply(p0);
        ov0 = m0.preMultiply(v0);
        assertEquals(op0[3], 3.42, kEpsilon);
        assertEquals(ov0[3], 2.42, kEpsilon);

        /* throws ...
        double p30[] = {1, 2, 3};
        op0 = m0.operate(p30);
        Logger.info(String.format("%g %g %g %g", 
                        op0[0], op0[1], op0[2], op0[3]));
         */

        Matrix4 mscale = Matrix4.identity();
        Matrix4 mtranslate = Matrix4.identity();
        double dscale[][] = mscale.getDataRef();
        double dxlate[][] = mtranslate.getDataRef();

        dscale[0][0] = 2;
        dscale[1][1] = 4;
        dscale[2][2] = 6;

        dxlate[0][3] = 3.33;
        dxlate[1][3] = 3.66;
        dxlate[2][3] = 3.99;

        /* for opencv rep, results translated, then scaled */
        Array2DRowRealMatrix product = mscale.multiply(mtranslate);
        Matrix4 mm = new Matrix4(product.getDataRef());
        double dprod[][] = mm.getDataRef();
        assertEquals(dprod[0][3], 6.66, kEpsilon);

        /* for opencv rep, results scale, then translate */
        product = mtranslate.multiply(mscale);
        mm = new Matrix4(product.getDataRef());
        dprod = mm.getDataRef();
        assertEquals(dprod[0][3], 3.33, kEpsilon);
    }

    @Test
    public void testQuaternion()
    {
        Quaternion q0 = new Quaternion(30, new Vec3(1,1,1));
        Quaternion q1 = new Quaternion(30, new Vec3(1,1,1).asUnit());
        // Logger.info("q0: " + q0.toString());
        // Quaternion 0.965926 0.149429 0.149429 0.149429
        assert(q1.equals(q0, kEpsilon));

        Matrix3 m30 = q0.asMatrix3();
        // Logger.info("m30: " + m30.toString());
        Quaternion q3 = new Quaternion(m30);
        assert(q3.equals(q0, kEpsilon));
    }
}