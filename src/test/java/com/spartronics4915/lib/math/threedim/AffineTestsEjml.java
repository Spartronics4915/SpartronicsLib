package com.spartronics4915.lib.math.threedim;

import com.spartronics4915.lib.math.threedim.ejml.*;

import java.lang.Math;
import java.util.ArrayList;
import java.util.Random;
import org.ejml.data.DMatrix4x4;
import org.ejml.data.DMatrix4;
import org.ejml.data.DMatrix3x3;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;

class AffineTestsEjml
{
    double kEpsilon = 1e-9;

    @Test
    void testVec3()
    {
        Vec3 aPt = new Vec3(1,2,3);
        Vec3 bPt = new Vec3(2,4,6);

        double aLen = aPt.length();
        assertEquals(aLen, Math.sqrt(1+4+9), kEpsilon);
        Vec3 nPt = new Vec3(aPt);
        nPt.normalize();
        assertEquals(nPt.length(), 1, kEpsilon);

        DMatrix3x3 op = aPt.outerProduct(bPt);
        // op.print(); // can be viewed if you "Run Test" in vscode, also in log
        assertEquals(op.a11, 2, kEpsilon);
        assertEquals(op.a22, 8, kEpsilon);
        assertEquals(op.a33, 18, kEpsilon);

        Vec3 cross = new Vec3(1,0,0).cross(new Vec3(0,1,0));
        assert(cross.equals(new Vec3(0,0,1), kEpsilon));

        Vec3 delta = new Vec3(1,1,1);
        Vec3 sub = cross.subtract(delta);
        Vec3 add = sub.add(delta);
        assert(add.equals(cross, kEpsilon));

        Vec3 a = new Vec3(1,1,1);
        Vec3 b = new Vec3(-1,-1,-1);
        assertEquals(a.angleWith(a), 0, kEpsilon);
        assertEquals(a.angleWith(b), 180, kEpsilon);
    }

    @Test
    void testAffine3()
    {
        Affine3 m = new Affine3();
        Vec3 aPt = new Vec3(1,2,3);

        // test for identity
        Vec3 v1 = m.transformPoint(aPt);
        assertEquals(v1.a1, aPt.a1);
        assertEquals(v1.a2, aPt.a2);
        assertEquals(v1.a3, aPt.a3);

        // test translation
        m = Affine3.fromTranslation(1,2,3);
        Vec3 v = m.getTranslation();
        assertEquals(v.a1, 1, kEpsilon);
        assertEquals(v.a2, 2, kEpsilon);
        assertEquals(v.a3, 3, kEpsilon);
        // m.print();

        Vec3 v2 = m.transformPoint(new Vec3(1,2,3));
        assertEquals(v2.a1, 2, kEpsilon);
        assertEquals(v2.a2, 4, kEpsilon);
        assertEquals(v2.a3, 6, kEpsilon);

        // simple rotation
        Affine3 R1 = Affine3.fromRotation(90, Vec3.ZAxis);
        Vec3 p1 = R1.transformVector(Vec3.XAxis);
        assert(p1.equals(Vec3.YAxis, kEpsilon));

        // rotation around the point [1,0,0]
        Affine3 R2 = Affine3.fromRotation(90, Vec3.ZAxis, new Vec3(1,0,0));
        Vec3 p2 = R2.transformPoint(Vec3.ZeroPt);
        assert(p2.equals(new Vec3(1, -1, 0), kEpsilon));
    }

    @Test
    public void testConcat()
    {
        // test concatenation
        Affine3 C1 = Affine3.fromRotation(15, Vec3.ZAxis);
        Affine3 C = new Affine3();
        // 6x15deg concats => 90 degree
        C.multiply(C1);
        C.multiply(C1);
        C.multiply(C1);
        C.multiply(C1);
        C.multiply(C1);
        C.multiply(C1);
        assert(C.equals(Affine3.fromRotation(90, Vec3.ZAxis), kEpsilon));

        // rotate before translate
        Vec3 diag = new Vec3(1,1,1);
        Affine3 T1 = Affine3.fromRotation(15, diag);
        T1.translate(3,4,5);
        Vec3 t1p = T1.transformPoint(Vec3.ZeroPt);
        assertEquals(t1p.a1, 3.183503, 1e-5);
        assertEquals(t1p.a2, 3.701142, 1e-5);
        assertEquals(t1p.a3, 5.115355, 1e-5);
        Vec3 t1d = T1.transformVector(diag);
        assert(t1d.equals(diag, kEpsilon));

        // translate before rotate
        Affine3 T2 = Affine3.fromTranslation(3,4,5);
        T2.rotate(15, new Vec3(1,1,1));
        Vec3 t2p = T2.transformPoint(Vec3.ZeroPt);
        assert(t2p.equals(new Vec3(3,4,5), kEpsilon));
        Vec3 t2d = T2.transformVector(diag);
        assert(t2d.equals(diag, kEpsilon));
    }

    @Test
    void testAffine3Rot()
    {
        Random rgen = new Random();
        double randAngle = 360 * (rgen.nextDouble()-.5); // [-180, 180]
        Vec3 randDir = new Vec3(rgen.nextDouble()-.5, rgen.nextDouble()-.5, 
                                rgen.nextDouble()-.5);
        randDir.normalize();
        Vec3 randPt = new Vec3(rgen.nextDouble()-.5, rgen.nextDouble()-.5, 
                                rgen.nextDouble()-.5);
        Affine3 R0 = Affine3.fromRotation(randAngle, randDir, randPt);
        Affine3 R0a = Affine3.fromRotation(randAngle-360, randDir, randPt); 
        assert(R0.equals(R0a, kEpsilon));

        Affine3 R0b = Affine3.fromRotation(randAngle, randDir, randPt);
        Affine3 R0c = Affine3.fromRotation(-randAngle, randDir.asOpposite(), randPt);
        assert(R0b.equals(R0c, kEpsilon));

        Affine3 I = new Affine3();
        Affine3 R0d = Affine3.fromRotation(360, randDir);
        assert(I.equals(R0d, kEpsilon));

        randDir = new Vec3(.32281269, -.08711045, -.46931146);
        randPt = new Vec3(-0.49508149, -0.10203337,  0.41560636);
        Affine3 R0e = Affine3.fromRotation(90, randDir.asUnit(), randPt);
        double t = R0e.trace();
        assert(Math.abs(2 - t) < kEpsilon);

        Affine3 A1 = Affine3.fromAxes(new Vec3(0, -1, 0), 
                                      new Vec3(0, 0, 1), 
                                      new Vec3(-1, 0, 0));
        ArrayList<Vec3> A1ret = A1.transformBases();
        Vec3[] A1ans = {new Vec3(0, -1, 0), new Vec3(0, 0, 1), new Vec3(-1, 0, 0)};
        int i=0;
        for(Vec3 v : A1ret)
        {
            assert(v.equals(A1ans[i++]));
        }
    }

    @Test
    void testQuaternion()
    {
        Quaternion q1 = new Quaternion(Math.toDegrees(.123), new Vec3(1, 0, 0));
        Quaternion q2 = new Quaternion(0.99810947, 0.06146124, 0, 0);
        assert(q1.equals(q2));
        
        Quaternion q3 = new Quaternion(new Affine3(), true);
        Quaternion q4 = new Quaternion();
        assert(q3.equals(q4));

        Affine3 a1 = Affine3.fromRotation(30, new Vec3(1,1,1));
        Quaternion q5 = new Quaternion(a1);
        String q5s = q5.asString();
        assertEquals(q5s, "q 0.965926 0.149429 0.149429 0.149429");
        // a1.print();
        Affine3 a2 = Affine3.fromQuaternion(q5);
        // a2.print();
        assert(a1.equals(a2, kEpsilon));
        
        /*
        >>> R = random_rotation_matrix()
        >>> q = quaternion_from_matrix(R)
        >>> is_same_transform(R, quaternion_matrix(q))
        True
        >>> is_same_quaternion(quaternion_from_matrix(R, isprecise=False),
        ...                    quaternion_from_matrix(R, isprecise=True))
        True
        >>> R = euler_matrix(0.0, 0.0, numpy.pi/2.0)
        >>> is_same_quaternion(quaternion_from_matrix(R, isprecise=False),
        ...                    quaternion_from_matrix(R, isprecise=True))
        True
        """
        */
    }
}