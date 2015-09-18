//#####################################################################
// Stanford CS148 Ray Tracer Example Code
// Copyright 2012, Ben Mildenhall
//#####################################################################

#ifndef RayTracer_Shape_h
#define RayTracer_Shape_h

#include "Ray.h"
#include "Intersection.h"
#include "AABB.h"

class Shape {
public:
    virtual bool getIntersect(const Ray& ray, Intersection* intersection) const = 0;
    virtual bool doesIntersect(const Ray& ray) const = 0;
    //virtual Intersection** getIntersections(const Ray& ray) = 0;
    //virtual bool isInsideClosed(const STPoint3& pt) = 0;
    //virtual bool isInsideOpen(const STPoint3& pt) = 0;
    virtual void getAABB(const STTransform4& transform, AABB* aabb) const = 0;
    
    virtual float getSurfaceArea() const { return -1.f; }
    virtual STPoint3 uniformSampleSurface(STVector3* normal) const { return STPoint3(0.f, 0.f, 0.f); }

    std::string name;
    //int maxInt;
};

#endif
