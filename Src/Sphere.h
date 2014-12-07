//#####################################################################
// Stanford CS148 Ray Tracer Example Code
// Copyright 2012, Ben Mildenhall
//#####################################################################

#ifndef RayTracer_Sphere_h
#define RayTracer_Sphere_h

#include "Shape.h"

class Sphere : public Shape {
public:
    Sphere(const STPoint3& _center, float _radius)
        : center(_center), radius(_radius) 
    {
        this->name = "sphere";
        //maxInt = 2;
    }
    Sphere(const Sphere& copy)
        : center(copy.center), radius(copy.radius)
    {
        this->name = copy.name;
        //this->maxInt = copy.maxInt;
    }

    void setCenter(const STPoint3& c) { center = c; }
    void setRadius(float r) { radius = r; }

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
    STPoint3 center;
    float radius;
};

#endif
