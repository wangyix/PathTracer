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
	//STPoint2 uv;
    Intersection(const float _t, const STPoint3& _point, const STVector3& _normal)//, STPoint2 _uv=STPoint2())
        : t(_t), point(_point), normal(_normal) {}
    Intersection(const Intersection& copy){ t = copy.t; point = copy.point; normal = copy.normal; }//uv = copy.uv; }
    Intersection() {}
    ~Intersection(){}
};

struct Vertex {
public:

    Vertex(const Intersection& inter, const Bsdf* bsdf, const SceneObject* obj) :
        w_to_prev(0.f),
        alpha(-1.f),
        G_prev(-1.f),
        qPsig_adj(-1.f),
        Pa_from_prev(-1.f),
        Pa_from_next(-1.f),
        prev_gap_nonspecular(-1.f),
        S(-1.f),
        bsdf(bsdf),
        obj(obj),
        intersection(inter)
    {
        // if the specified bsdf is expecting its params wi, wo in normal-space, construct
        // axes IJK with K aligned with the intersection normal and I,J in some arbitrary rotated
        // configuration around K.  calculate world-to-normal and normal-to-world transforms using IJK.
        if (!bsdf->worldSpaceParams()) {
            const STVector3& n = intersection.normal;
            const STPoint3& P = intersection.point;

            // generate unit axes I,J,K  where K=normal;
            // I,J will be set to some arbitrary orientation around K
            STVector3 I, J;
            STVector3 K = n;
            if (K.x() == 0.f && K.y() == 0.f) {
                I = STVector3(1.f, 0.f, 0.f);
                J = STVector3(0.f, 1.f, 0.f);
            } else {
                I = K.cross3(STVector3(0.f, 0.f, 1.f)); //STVector3::Cross(K, STVector3(0.f, 0.f, 1.f));
                I.normalize();
                J = K.cross3(I);     //STVector3::Cross(K, I);
                J.normalize();
            }
            // generate transform matrices between normal- and world-space from I,J,K axes
            normalToWorld = STTransform4();
            normalToWorld(0, 0) = I.x();
            normalToWorld(1, 0) = I.y();
            normalToWorld(2, 0) = I.z();
            normalToWorld(3, 0) = 0.f;
            normalToWorld(0, 1) = J.x();
            normalToWorld(1, 1) = J.y();
            normalToWorld(2, 1) = J.z();
            normalToWorld(3, 1) = 0.f;
            normalToWorld(0, 2) = K.x();
            normalToWorld(1, 2) = K.y();
            normalToWorld(2, 2) = K.z();
            normalToWorld(3, 2) = 0.f;
            normalToWorld(0, 3) = P.x();
            normalToWorld(1, 3) = P.y();
            normalToWorld(2, 3) = P.z();
            normalToWorld(3, 3) = 1.f;

            /*normalToWorld = STTransform4(
                I.x, J.x, K.x, P.x,
                I.y, J.y, K.y, P.y,
                I.z, J.z, K.z, P.z,
                0.f, 0.f, 0.f, 1.f
                );*/
            worldToNormal = normalToWorld.inverse();
        } else {
            normalToWorld = STTransform4::Identity();
            worldToNormal = STTransform4::Identity();
        }
    }

    const Intersection& getIntersection() const { return intersection; }

    // wrappers around the Bsdf versions;
    // wo and wi are given/returned in world-space instead of normal-space
    STColor3f f(const STVector3& wo_w, const STVector3& wi_w) const {
        STVector3 wo = worldToNormal * wo_w;
        STVector3 wi = worldToNormal * wi_w;
        return bsdf->f(wo, wi);
    }
    STColor3f sample_f(const STVector3& wo_w, STVector3* wi_w, float* pdf_sig, float* cos_wi) const {
        STVector3 wo = worldToNormal * wo_w;
        STVector3 wi;
        STColor3f f = bsdf->sample_f(wo, &wi, pdf_sig, cos_wi);
        *wi_w = normalToWorld * wi;
        return f;
    }
    float p_sig(const STVector3& wo_w, const STVector3& wi_w) const {
        STVector3 wo = worldToNormal * wo_w;
        STVector3 wi = worldToNormal * wi_w;
        return bsdf->p_sig(wo, wi);
    }
    bool isSpecular() const {
        return bsdf->isSpecular();
    }

    std::string getBsdfDescriptionString() const {
        return bsdf->getDescriptionString();
    }

    const SceneObject* getObj() const {
        return obj;
    }

public:
    STVector3 w_to_prev;       // direction to previous vertex
    STColor3f alpha;        // alpha_i1
    float G_prev;           // G(zi->z1i) 
    float qPsig_adj;        // q*Psig(zi->zi1) = q*Psig(zi->z_1i)

    // these 3 members should eliminate special cases when calculating S values
    float Pa_from_prev;          // Pa(z1i->zi)
    float Pa_from_next;         // Pa(zi1->zi)
    float prev_gap_nonspecular; // false if zi or z1i has specular Psig

    float S;                // (pi/pi1)^2 + ... + (p0/pi1)^2, terms corresponding to specular-gap are 0

private:
    const Bsdf* bsdf;       // used for isSpecular, f( ), and Psig( ). may be different from obj->bsdf (i.e. in case of lights, bsdf=y0Lambertian instead of obj->bsdf)
    const SceneObject* obj;
    Intersection intersection;
    STTransform4 normalToWorld, worldToNormal;  // transforms between world-space and normal-space
};

#endif
