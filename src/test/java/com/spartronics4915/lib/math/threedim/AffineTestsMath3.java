package com.spartronics4915.lib.math.threedim;

import com.spartronics4915.lib.math.threedim.math3.*;

import java.lang.Math;
import java.util.ArrayList;
import java.util.Random;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;

class AffineTestsMath3
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

        Matrix3 op = aPt.outerProduct(bPt);
        // op.print(); // can be viewed if you "Run Test" in vscode, also in log
        assertEquals(op.getEntry(0,0), 2, kEpsilon);
        assertEquals(op.getEntry(1,1), 8, kEpsilon);
        assertEquals(op.getEntry(2,2), 18, kEpsilon);

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
        assert(v1.equals(aPt));

        // test translation
        m = Affine3.fromTranslation(1,2,3);
        Vec3 v = m.getTranslation();
        assert(v.equals(new Vec3(1,2,3)));

        Vec3 v2 = m.transformPoint(new Vec3(1,2,3));
        assert(v2.equals(new Vec3(2,4,6)));

        // simple rotation
        Affine3 R1 = Affine3.fromRotation(90, Vec3.ZAxis);
        Vec3 p1 = R1.transformVector(Vec3.XAxis);
        assert(p1.equals(Vec3.YAxis, kEpsilon));

        // simple rotation around the point [1,0,0]
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
        Affine3 C1Cum = Affine3.fromRotation(90, Vec3.ZAxis);
        assert(C.equals(C1Cum, kEpsilon));

        // rotate before translate
        Vec3 diag = new Vec3(1,1,1);
        Affine3 T1 = Affine3.fromRotation(15, diag);
        T1.translate(3,4,5);
        Vec3 t1p = T1.transformPoint(Vec3.ZeroPt);
        assertEquals(t1p.getX(), 3.183503, 1e-5);
        assertEquals(t1p.getY(), 3.701142, 1e-5);
        assertEquals(t1p.getZ(), 5.115355, 1e-5);
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
        
        Affine3 a1 = new Affine3();
        Quaternion q3 = new Quaternion(a1.asMatrix(), true);
        Quaternion q4 = new Quaternion();
        assert(q3.equals(q4));

        Affine3 a2 = Affine3.fromRotation(30, new Vec3(1,1,1));
        Quaternion q5 = new Quaternion(a2.asMatrix(), true);
        String q5s = q5.toString();
        assertEquals(q5s, "Q 0.965926 0.149429 0.149429 0.149429");
        // a2.print();
        Affine3 a3 = Affine3.fromQuaternion(q5);
        // a3.print();
        assert(a3.equals(a2, kEpsilon));
    }
}