#include "Bsdf.h"
#include "STColor3f.h"
#include "STVector3.inl"

#define _USE_MATH_DEFINES
#include <math.h>

#include <cstdlib>


STColor3f FrDiel(float cosi, float cost, const STColor3f& etai, const STColor3f& etat) {
    STColor3f Rparl = ((etat * cosi) - (etai * cost)) /
        ((etat * cosi) + (etai * cost));
    STColor3f Rperp = ((etai * cosi) - (etat * cost)) /
        ((etai * cosi) + (etat * cost));
    return (Rparl*Rparl + Rperp*Rperp) / 2.f;
}

STColor3f FrCond(float cosi, const STColor3f& eta, const STColor3f& k) {
    STColor3f tmp = (eta*eta + k*k) * cosi*cosi;
    STColor3f Rparl2 = (tmp - (2.f * eta * cosi) + 1) /
        (tmp + (2.f * eta * cosi) + 1);
    STColor3f tmp_f = eta*eta + k*k;
    STColor3f Rperp2 =
        (tmp_f - (2.f * eta * cosi) + cosi*cosi) /
        (tmp_f + (2.f * eta * cosi) + cosi*cosi);
    return (Rparl2 + Rperp2) / 2.f;
}

// code from FresnelDielectric::Evaluate()
STColor3f fresnelDielEvaluate(float cosi, float etai, float etat) {
    // Compute Fresnel reflectance for dielectric
    cosi = std::min(std::max(cosi, -1.f), 1.f);

    // Compute indices of refraction for dielectric
    bool entering = cosi > 0.;
    float ei = etai, et = etat;
    if (!entering)
        std::swap(ei, et);

    // Compute _sint_ using Snell's law
    float sint = ei / et * sqrtf(std::max(0.f, 1.f - cosi*cosi));
    if (sint >= 1.) {
        // Handle total internal reflection
        return STColor3f(1.f);
    } else {
        float cost = sqrtf(std::max(0.f, 1.f - sint*sint));
        return FrDiel(fabsf(cosi), cost, STColor3f(ei), STColor3f(et));
    }
}






STColor3f Lambertian::f(const STVector3& wo, const STVector3& wi) const {
    return R / M_PI;
}

STColor3f Lambertian::sample_f(const STVector3& wo, STVector3* wi, float *pdf_sig) const {
    // cosine-sample the hemisphere
    float r = (float)rand() / RAND_MAX;                     // [0, 1]
    float theta = (float)rand() / RAND_MAX * 2.0f * M_PI;   // [0, 2pi]
    float sqrt_r = sqrtf(r);
    wi->x = sqrt_r * cosf(theta);
    wi->y = sqrt_r * sinf(theta);
    wi->z = sqrtf(std::max(0.f, 1.f - wi->x*wi->x - wi->y*wi->y));
    if (CosTheta(wo) < 0.f) wi->z = -wi->z;     // make sure wi, wo are in same hemisphere

    *pdf_sig = 1.0f / M_PI;
    return f(wo, *wi);
}


STColor3f SpecularDiel::f(const STVector3& wo, const STVector3& wi) const {
    return STColor3f(0.f);
}

STColor3f SpecularDiel::sample_f(const STVector3& wo, STVector3* wi, float *pdf_sig) const {
    // based on dielectric reflectance, randomly select the transmitted ray 
    // or the reflected ray. Have q be probability of selecting reflected ray
    STColor3f F = fresnelDielEvaluate(CosTheta(wo), etai, etat);
    float q = (F.r + F.g + F.b) / 3.0f;
    if ((float)rand() / RAND_MAX <= q) {
        // select reflected ray
        *pdf_sig = q;

        // similar code to SpecularCond::samle_f, except reflectance F
        // is calculated using dielectric equation instead of conductor
        {
            // wi will be wo reflected
            wi->x = -wo.x;
            wi->y = -wo.y;
            wi->z = wo.z;
            //*pdf_sig = 1.0f;

            // bsdf is Fr * delta(w-wi) / cos(thetai)
            return R * F / AbsCosTheta(*wi);
        }
    } else {
        // select transmitted ray
        *pdf_sig = 1.f - q;
        
        // code from pbrt SpecularTransmission::Samle_f
        {
            // figure out which eta is incident and which is transmitted
            bool entering = CosTheta(wo) > 0.f;
            float ei = etai, et = etat;
            if (!entering)
                std::swap(ei, et);

            // compute transmitted ray direction
            float sini2 = SinTheta2(wo);
            float eta = ei / et;
            float sint2 = eta * eta * sini2;

            // Handle total internal reflection for transmission
            if (sint2 >= 1.) return STColor3f(0.f);
            float cost = sqrtf(std::max(0.f, 1.f - sint2));
            if (entering) cost = -cost;
            float sintOverSini = eta;
            *wi = STVector3(sintOverSini * -wo.x, sintOverSini * -wo.y, cost);
            //*pdf_sig = 1.f;
            //STColor3f F = fresnelDielEvaluate(CosTheta(wo), etai, etat);
            return /*(ei*ei)/(et*et) * */ (STColor3f(1.) - F) * T /
                AbsCosTheta(*wi);
        }
    }
}

STColor3f SpecularCond::f(const STVector3& wo, const STVector3& wi) const {
    return STColor3f(0.f);
}

STColor3f SpecularCond::sample_f(const STVector3& wo, STVector3* wi, float *pdf_sig) const {
    // wi will always be wo reflected
    wi->x = -wo.x;
    wi->y = -wo.y;
    wi->z = wo.z;
    *pdf_sig = 1.0f;     // 1*dirac

    // bsdf is Fr * delta(w-wi) / cos(thetai)
    return FrCond(AbsCosTheta(wo), eta, k) * R / AbsCosTheta(*wi);
}

STColor3f ScaledBsdf::f(const STVector3& wo, const STVector3& wi) const {
    return s * bsdf->f(wo, wi);
}

STColor3f ScaledBsdf::sample_f(const STVector3& wo, STVector3* wi, float *pdf_sig) const {
    return s * bsdf->sample_f(wo, wi, pdf_sig);
}