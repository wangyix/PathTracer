//#####################################################################
// Stanford CS148 Ray Tracer Example Code
// Copyright 2012, Ben Mildenhall
//#####################################################################

#ifndef RayTracer_Ray_h
#define RayTracer_Ray_h

#include "float.h"
#include "STVector3.h"
#include "STPoint3.h"
#include "STTransform4.h"

class SceneObject;

class Ray {
public:
    Ray() : t_min(0.f), t_max(FLT_MAX), e_obj(NULL){}
    
    // for compatibility with existing code
    Ray(const STPoint3& _e, const STVector3& _d, float _t_min = 0.f, float _t_max = FLT_MAX)
        : e(_e), d(_d), e_obj(NULL), d_dot_e_normal(0.f), t_min(_t_min), t_max(_t_max) {}

    Ray(const STPoint3& _e, const STVector3& _d, const SceneObject* _e_obj, const STVector3& _e_normal,
        float _t_min = 0.f, float _t_max = FLT_MAX)
        : e(_e), d(_d), e_obj(_e_obj), d_dot_e_normal(_d.dot( _e_normal)), t_min(_t_min), t_max(_t_max) {}

    Ray(const STPoint3& _e, const STVector3& _d, const SceneObject* _e_obj, float _d_dot_e_normal,
        float _t_min = 0.f, float _t_max = FLT_MAX)
        : e(_e), d(_d), e_obj(_e_obj), d_dot_e_normal(_d_dot_e_normal), t_min(_t_min), t_max(_t_max) {}
    
    bool inRange(float t) const { return t >= t_min && t <= t_max; }
    STPoint3 at(float t) const { return e + d * t ; }
    Ray transform(const STTransform4& M) const {
        return Ray(M * e, M * d, e_obj, d_dot_e_normal, t_min, t_max);
    }
    
    STPoint3 e;
    STVector3 d;
    const SceneObject* e_obj;
    //STVector3 e_normal;
    float d_dot_e_normal;
    float t_min;
    float t_max;
};

#endif
