//#####################################################################
// Stanford CS148 Ray Tracer Example Code
// Copyright 2012, Ben Mildenhall
//#####################################################################

#include "Camera.h"

static const double pi = 3.14159265358979;

Camera::Camera(const STPoint3& _eye, const STVector3& _up, const STPoint3& _lookAt, float _fovy, float _aspect)
: eye(_eye), up(_up), lookAt(_lookAt), fovy(_fovy), aspect(_aspect)
{
    left = STVector3::Cross(up, lookAt - eye);
    up = STVector3::Cross(lookAt - eye, left);
    left.Normalize();
    up.Normalize();
    
    float y = (lookAt - eye).Length() * tan(.5f * (fovy * (float)pi / 180.f));
    float x = y * aspect;
    UL = STVector3(lookAt) + x * left + y * up;
    UR = STVector3(lookAt) - x * left + y * up;
    LL = STVector3(lookAt) + x * left - y * up;
    LR = STVector3(lookAt) - x * left - y * up;
    
}

STPoint3 Camera::pointOnPlane(float u, float v) const {
    STVector3 pVec = (1 - u) * ((1 - v) * LL + v * UL)
                        + u * ((1 - v) * LR + v * UR);
    return STPoint3(pVec);
}

Ray* Camera::generateRay(float u, float v, float bias) const {
    return new Ray(eye, pointOnPlane(u, v) - eye, bias);
}

void Camera::generateRay(Ray& ray, float u, float v, float bias) const {
    ray = Ray(eye, pointOnPlane(u, v) - eye, bias);
}

float Camera::getFocalRatio(const STPoint3 &f) {
    return STVector3::Dot(f - eye, lookAt - eye) / (lookAt - eye).LengthSq();
}

// calculates Psig(z0->z1) assuming pinhole camera.
// Psig(z0->z1) = 0 when z1 not in img plane; this assumes (u,v) is in img plane
// Psig(z0->z1) = 1 / (4 * a * tan(fovy/2)^2 * cos(theta)^4)
float Camera::Psig(float u, float v) const {
    // convert (u,v) to half-NDC coordinates (s,t) which are [-0.5, 0.5]
    float s = u - 0.5f;
    float t = v - 0.5f;

    float tanHalfFovy = tanf(0.5f * fovy * (float)pi / 180.f);
    float d = 0.5f / tanHalfFovy;   // eye distance to img plane if height=1
    float as = aspect * s;
    float cosThetaSq = d*d / (as*as + t*t + d*d);
    float tanHalfFovy_cosThetaSq = tanHalfFovy * cosThetaSq;
    return 1.0f / (4.0f * aspect * tanHalfFovy_cosThetaSq * tanHalfFovy_cosThetaSq);
}
