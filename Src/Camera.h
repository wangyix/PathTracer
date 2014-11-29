//#####################################################################
// Stanford CS148 Ray Tracer Example Code
// Copyright 2012, Ben Mildenhall
//#####################################################################

#ifndef RayTracer_Camera_h
#define RayTracer_Camera_h

#include "STPoint3.h"
#include "STVector3.h"
#include "STColor3f.h"
#include "Ray.h"
#include "Bsdf.h"


class Camera;

class CameraBsdf : public Bsdf {
public:
    CameraBsdf(const Camera& camera) : camera(camera), u_sample(-1.f), v_sample(-1.f) {}
    STColor3f f(const STVector3& wo, const STVector3& wi) const;
    STColor3f sample_f(const STVector3& wo, STVector3* wi, float *pdf_sig, float* cos_wi) const;
    float p_sig(const STVector3& wo, const STVector3& wi) const;
    bool isSpecular() const { return false; }

    void setSampleUV(float u, float v);

private:
    const Camera& camera;

    // set externally so that the w corrsponding to this (u,v) is chosen when
    // sample_f is called.
    float u_sample, v_sample;
};


class Camera {
    friend class CameraBsdf;
public:
    Camera(const STPoint3& eye, const STVector3& up, const STPoint3& lookAt, float fovy, float aspect);
    ~Camera(){}
    Ray* generateRay(float u, float v, float bias = 0.) const;
    void generateRay(Ray& ray, float u, float v, float bias = 0.) const;
    float getFocalRatio(const STPoint3& f);
    STPoint3 pointOnPlane(float u, float v) const;

    const STPoint3& getEye() const { return eye; }
    STVector3 getLook() const {
        STVector3 look = lookAt - eye;
        look.Normalize();
        return look;
    }

    void getUvOfDirection(const STVector3 w, float* u, float* v) const;

    void setSampleUV(float u, float v);
    void sample_z0(STPoint3* z0, STVector3* z0_n, Bsdf const** bsdf, float* Pa, STColor3f* We0) const;

private:
    STPoint3 eye;
    STVector3 up;
    STPoint3 lookAt;
    float fovy;
    float aspect;
    
    STVector3 left, UL, UR, LL, LR;

    STTransform4 worldToView;
    float LR_half_dist;
    float UL_half_dist;

    CameraBsdf cameraBsdf;
};

#endif
