//#####################################################################
// Stanford CS148 Ray Tracer Example Code
// Copyright 2012, Ben Mildenhall
//#####################################################################

#ifndef RayTracer_Intersection_h
#define RayTracer_Intersection_h

#include "STPoint3.h"
#include "STPoint2.h"
#include "STVector3.h"
#include "Bsdf.h"

struct Intersection {
    float t;
    STPoint3 point;
    STVector3 normal;
	STPoint2 uv;
    Intersection(const float _t, const STPoint3& _point, const STVector3& _normal,const STPoint2 _uv=STPoint2()): t(_t), point(_point), normal(_normal), uv(_uv) {}
    Intersection(const Intersection& copy){t=copy.t;point=copy.point;normal=copy.normal;uv=copy.uv;}
    Intersection() {}
    ~Intersection(){}
};

struct Vertex {
public:

    Vertex(const Intersection& inter) :
        bsdf(NULL),
        w_prev(0.f),
        alpha(-1.f),
        G_prev(-1.f),
        qPsig_adj(-1.f)
    {
        intersection = inter;
        const STVector3& n = intersection.normal;
        const STPoint3& P = intersection.point;

        // generate unit axes I,J,K  where K=normal;
        // I,J will be set to some arbitrary orientation around K
        STVector3 I, J;
        STVector3 K = n;
        if (K.x == 0.f & K.y == 0.f) {
            I = STVector3(1.f, 0.f, 0.f);
            J = STVector3(0.f, 1.f, 0.f);
        } else {
            I = STVector3::Cross(K, STVector3(0.f, 0.f, 1.f));
            I.Normalize();
            J = STVector3::Cross(K, I);
            J.Normalize();
        }
        // generate transform matrices between normal- and world-space from I,J,K axes
        normalToWorld = STTransform4(
            I.x, J.x, K.x, P.x,
            I.y, J.y, K.y, P.y,
            I.z, J.z, K.z, P.z,
            0.f, 0.f, 0.f, 1.f
            );
        worldToNormal = normalToWorld.Inverse();
    }

    const Intersection& getIntersection() const { return intersection; }

    // wrappers around the Bsdf versions;
    // wo and wi are given/returned in world-space instead of normal-space
    STColor3f f(const STVector3& wo_w, const STVector3& wi_w) const {
        STVector3 wo = worldToNormal * wo_w;
        STVector3 wi = worldToNormal * wi_w;
        return bsdf->f(wo, wi);
    }
    STColor3f sample_f(const STVector3& wo_w, STVector3* wi_w, float* pdf_sig,
        float* cos_wi) const {
        STVector3 wo = worldToNormal * wo_w;
        STVector3 wi;
        STColor3f f = bsdf->sample_f(wo, &wi, pdf_sig);
        *cos_wi = AbsCosTheta(wi);
        *wi_w = normalToWorld * wi;
        return f;
    }

public:
    const Bsdf* bsdf;   // used for isSpecular, f( ), and Psig( )
    STVector3 w_prev;   // direction to previous vertex
    STColor3f alpha;    // alpha_i1
    float G_prev;       // G(zi->z1i) 

    float qPsig_adj;    // q*Psig(zi->zi1) = q*Psig(zi->z_1i)

private:
    Intersection intersection;
    STTransform4 normalToWorld, worldToNormal;  // transforms between world-space and normal-space
};

#endif
