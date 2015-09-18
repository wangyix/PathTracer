//#####################################################################
// Stanford CS148 Ray Tracer Example Code
// Copyright 2012, Ben Mildenhall
//#####################################################################

#ifndef RayTracer_Cylinder_h
#define RayTracer_Cylinder_h

#include "Shape.h"
#include "Triangle.h"

class Cylinder : public Shape {
public:
#if USE_EIGEN
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
    Cylinder(const STPoint3& _A, const STPoint3& _B, float _radius) {
        this->name = "cylinder";
        //maxInt = 2;
        setAttributes(_A, _B, _radius);
    }
    Cylinder(const Cylinder& copy)
    {
        this->name=copy.name;
        //this->maxInt=copy.maxInt;
        setAttributes(copy.A, copy.B, copy.radius);
    }

    void setAttributes(const STPoint3& _A, const STPoint3& _B, float _radius) {
        A = _A;
        B = _B;
        radius = _radius;

        // construct axes for cylinder (cylinder along z axis, from 0 to |A-B|
        // I, J are radius length, perpendicular to K, which is equal to B-A;
        STVector3 I, J;
        STVector3 K = B - A;
        if (K.x() == 0.f && K.y() == 0.f) {
            I = STVector3(radius, 0.f, 0.f);
            J = STVector3(0.f, radius, 0.f);
        } else {
            I = K.cross3(STVector3(0.f, 0.f, 1.f));  // STVector3::Cross(K, STVector3(0.f, 0.f, 1.f));
            I.normalize();
            I *= radius;
            J = K.cross3(I); // STVector3::Cross(K, I);
            J.normalize();
            J *= radius;
        }

        toObjectSpace(0, 0) = I.x();
        toObjectSpace(1, 0) = I.y();
        toObjectSpace(2, 0) = I.z();
        toObjectSpace(3, 0) = 0.f;
        toObjectSpace(0, 1) = J.x();
        toObjectSpace(1, 1) = J.y();
        toObjectSpace(2, 1) = J.z();
        toObjectSpace(3, 1) = 0.f;
        toObjectSpace(0, 2) = K.x();
        toObjectSpace(1, 2) = K.y();
        toObjectSpace(2, 2) = K.z();
        toObjectSpace(3, 2) = 0.f;
        toObjectSpace(0, 3) = A.x();
        toObjectSpace(1, 3) = A.y();
        toObjectSpace(2, 3) = A.z();
        toObjectSpace(3, 3) = 1.f;
        /*toObjectSpace = STTransform4(
            I.x, J.x, K.x, A.x,
            I.y, J.y, K.y, A.y,
            I.z, J.z, K.z, A.z,
            0.f, 0.f, 0.f, 1.f
        );*/
        toUnitCylinderSpace = toObjectSpace.inverse();
        toObjectInvTranspose = toUnitCylinderSpace.transpose();
    }

    void getAABB(const STTransform4& transform, AABB* aabb) const override;

    bool getIntersect(const Ray& ray, Intersection* intersection) const override;
    bool doesIntersect(const Ray& ray) const override;

    float getSurfaceArea() const override;
    STPoint3 uniformSampleSurface(STVector3* normal) const override;

private:
    STTransform4 toUnitCylinderSpace;
    STTransform4 toObjectSpace;
    STTransform4 toObjectInvTranspose;

    STPoint3 A, B;
    float radius;
};

#endif
