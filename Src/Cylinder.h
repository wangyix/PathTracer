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
        if (K.x == 0.f && K.y == 0.f) {
            I = STVector3(radius, 0.f, 0.f);
            J = STVector3(0.f, radius, 0.f);
        } else {
            I = STVector3::Cross(K, STVector3(0.f, 0.f, 1.f));
            I.Normalize();
            I *= radius;
            J = STVector3::Cross(K, I);
            J.Normalize();
            J *= radius;
        }

        toObjectSpace = STTransform4(
            I.x, J.x, K.x, A.x,
            I.y, J.y, K.y, A.y,
            I.z, J.z, K.z, A.z,
            0.f, 0.f, 0.f, 1.f
        );
        toUnitCylinderSpace = toObjectSpace.Inverse();
        toObjectInvTranspose = toUnitCylinderSpace.Transpose();
    }

    /*~Cylinder()
    {
        delete top;delete bottom;
    }*/

    /*Intersection* getIntersect(const Ray& ray);
    bool doesIntersect(const Ray& ray);
    Intersection** getIntersections(const Ray& ray);
    bool isInsideOpen(const STPoint3& pt);
    bool isInsideClosed(const STPoint3& pt);
    AABB* getAABB();*/

    bool getIntersect(const Ray& ray, Intersection* intersection) const override;
    bool doesIntersect(const Ray& ray) const override;

    float getSurfaceArea() const override;
    STPoint3 uniformSampleSurface(STVector3* normal) const override;

private:
    STPoint3 A, B;
    float radius;

    STTransform4 toUnitCylinderSpace;
    STTransform4 toObjectSpace;
    STTransform4 toObjectInvTranspose;

    //Triangle top, bottom;
    //bool isWithinCylinder(const STPoint3& pt);
    //Intersection* closer(Intersection* a, Intersection* b);
};

#endif
