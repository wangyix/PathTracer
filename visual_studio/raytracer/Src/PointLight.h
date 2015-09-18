//#####################################################################
// Stanford CS148 Ray Tracer Example Code
// Copyright 2012, Ben Mildenhall
//#####################################################################

#ifndef RayTracer_PointLight_h
#define RayTracer_PointLight_h

#include "Light.h"
#include <iostream>

class PointLight : public Light {
public:
    PointLight(const STPoint3& _point, const STColor3f& _col)
    : point(_point), col(_col) { name = "point"; }
    
    STColor3f color(Intersection *inter, const STVector3& view, const STColor3f& amb, const STColor3f& diff, const STColor3f& spec, float shine) override {
		STVector3 L = point - inter->point;
		L.normalize();
		STVector3 N = inter->normal;
		STVector3 R = L.dot(N) * 2 * N - L;
		return diff * col * clamp(L.dot(N)) + spec * col * pow(clamp(R.dot(view.normalized())), shine);
    }
    
    STVector3 direction(const STPoint3& pt) { return point - pt; }

    STColor3f col;
    STPoint3 point;
};


#endif
