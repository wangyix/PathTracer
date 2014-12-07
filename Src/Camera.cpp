//#####################################################################
// Stanford CS148 Ray Tracer Example Code
// Copyright 2012, Ben Mildenhall
//#####################################################################

#include "Camera.h"

static const double pi = 3.14159265358979;

Camera::Camera()
    : eye(STPoint3(0.f, 0.f, 0.f)), up(STVector3(0.f, 1.f, 0.f)), lookAt(0.f, 0.f, -1.f), fovy(45.f), aspect(1.f), cameraBsdf(*this)
{
}

Camera::Camera(const STPoint3& _eye, const STVector3& _up, const STPoint3& _lookAt, float _fovy, float _aspect)
    : cameraBsdf(*this)
{
    setAttributes(_eye, _up, _lookAt, _fovy, _aspect);
}

void Camera::setAttributes(const STPoint3& _eye, const STVector3& _up, const STPoint3& _lookAt, float _fovy, float _aspect) {
    eye = _eye;
    up = _up;
    lookAt = _lookAt;
    fovy = _fovy;
    aspect = _aspect;

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
    UL_half_dist = y;
    LR_half_dist = x;

    // calculate world-to-view matrix
    const STVector3& I = left;
    const STVector3& J = up;
    STVector3 K = getLook();
    const STPoint3& P = eye;
    STTransform4 viewToWorld = STTransform4(
        I.x, J.x, K.x, P.x,
        I.y, J.y, K.y, P.y,
        I.z, J.z, K.z, P.z,
        0.f, 0.f, 0.f, 1.f
        );
    worldToView = viewToWorld.Inverse();
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


void Camera::getUvOfDirection(const STVector3 w, float* u, float* v) const {
    STVector3 w_v = worldToView * w;
    
    // scale w_v so that it goes from the eye to a point on the camera plane
    float target_z = (lookAt - eye).Length();
    STVector3 w_v_scaled = w_v * (target_z / w_v.z);
    float ndc_u = w_v_scaled.x / LR_half_dist;  // [-1, 1]
    float ndc_v = w_v_scaled.y / UL_half_dist;  // [-1, 1]

    *u = .5f * (1.f - ndc_u);
    *v = .5f * (ndc_v + 1.f);
}


void Camera::setSampleUV(float u, float v) {
    cameraBsdf.setSampleUV(u, v);
}

// technically, to achieve our target of We(x) = delta(x-xe)/cos(w)^3, we should have:
// We0(x) = C*delta(x-xe)
// We1(xe, w) =  1/C * 1/cos(w)^3, where C = integral of We1(xe, w)dsig(w) over the img plane
// However, any C should work.

// note that f/Psig = (4*a*tan(fovy/2)^2 / C) * cosw.  We would like to set C so that f/Psig
// is close to or above 1 for directions w in the img plane to decrease the odds of the eye-subpath
// terminating at z0. So we want C to be some small number.

// 4*(16/9)*tan(45/2)^2 * cos(60) = 0.6100, so any C below that should result in f/Psig > 1
// for most cases

#define C 0.1f

void Camera::sample_z0(STPoint3* z0, STVector3* z0_n, Bsdf const** bsdf, float* Pa, STColor3f* We0) const {
    *z0 = eye;
    *z0_n = getLook();
    *bsdf = &cameraBsdf;
    *Pa = 1.f;              // Pa(z0) = delta()
    *We0 = STColor3f(C);  // should be C since We1 should be 1/C, but 1 should work
}






// The wrapper functions f(), sample_f(), p_sig() in class Vertex will be calling the next
// three functions with the expectation that wo,wi are in world-space

STColor3f CameraBsdf::f(const STVector3& wo, const STVector3& wi) const {
    // f here means We_1(z0, w), which is 1/cosw^3 to offset the cos terms in the solid angle
    // subtended by pixels on the img plane.
    // Should really be scaled by 1/C so We_1(z0, w) integrates to 1, but this should work
    float cos_wi = STVector3::Dot(wi, camera.getLook());
    return STColor3f(1.f) / (C * cos_wi * cos_wi * cos_wi);
}

STColor3f CameraBsdf::sample_f(const STVector3& wo, STVector3* wi, float *pdf_sig, float* cos_wi) const {
    // we'll choose w so that it goes thru (u_sample,v_sample), as we were told
    Ray w_ray;
    camera.generateRay(w_ray, u_sample, v_sample);
    w_ray.d.Normalize();
    *wi = w_ray.d;
    *cos_wi = STVector3::Dot(*wi, camera.getLook());

    *pdf_sig = p_sig(wo, *wi);
    return f(wo, *wi);
}

float CameraBsdf::p_sig(const STVector3& wo, const STVector3& wi) const {
    // Psig(z0->z1) = 0 when z1 not in img plane; this assumes wi is in img plane
    // Psig(z0->z1) = 1 / (4 * a * tan(fovy/2)^2 * cos(theta)^4)

    STVector3 look = camera.getLook();
    float cos_wi = STVector3::Dot(wi, look);

    float tanHalfFovy = tanf(0.5f * camera.fovy * (float)pi / 180.f);
    float tanHalfFovy_cosThetaSq = tanHalfFovy * cos_wi * cos_wi;
    return 1.f / (4.f * camera.aspect * tanHalfFovy_cosThetaSq * tanHalfFovy_cosThetaSq);
}

void CameraBsdf::setSampleUV(float u, float v) {
    u_sample = u;
    v_sample = v;
}
