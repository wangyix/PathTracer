#include "Bsdf.h"
#include "STColor3f.h"
#include "STUtil.h"

#include <sstream>
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

/*// code from FresnelDielectric::Evaluate()
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
}*/






STColor3f Lambertian::f(const STVector3& wo, const STVector3& wi) const {
    if ((wo.z() >= 0.f) == (wi.z() >= 0.f)) {
        return R / M_PI;
    }
    return STColor3f(0.f);
}

STColor3f Lambertian::sample_f(const STVector3& wo, STVector3* wi, float *pdf_sig, float* cos_wi) const {
    // cosine-sample the hemisphere
    float r = randFloat();                     // [0, 1]
    float theta = randFloat() * 2.0f * M_PI;   // [0, 2pi]
    float sqrt_r = sqrtf(r);
    wi->x() = sqrt_r * cosf(theta);
    wi->y() = sqrt_r * sinf(theta);
    wi->z() = sqrtf(std::max(0.f, 1.f - wi->x()*wi->x() - wi->y()*wi->y()));
    if (CosTheta(wo) < 0.f) wi->z() = -wi->z();     // make sure wi, wo are in same hemisphere

    *pdf_sig = 1.0f / M_PI;
    *cos_wi = AbsCosTheta(*wi);
    return f(wo, *wi);
}

float Lambertian::p_sig(const STVector3& wo, const STVector3& wi) const {
    if ((wo.z() >= 0.f) == (wi.z() >= 0.f)) {
        return 1.f / M_PI;
    }
    return 0.f;
}

std::string Lambertian::getDescriptionString() const {
    std::stringstream ss;
    ss << "Lambertian  " << R.r << " " << R.g << " " << R.b;
    return ss.str();
}


STColor3f Y0Lambertian::f(const STVector3& wo, const STVector3& wi) const {
    if (wi.z() >= 0.f) {
        return STColor3f(1.f / M_PI);
    }
    return STColor3f(0.f);
}

STColor3f Y0Lambertian::sample_f(const STVector3& wo, STVector3* wi, float *pdf_sig, float* cos_wi) const {
    // cosine-sample the hemisphere
    float r = randFloat();                     // [0, 1]
    float theta = randFloat() * 2.0f * M_PI;   // [0, 2pi]
    float sqrt_r = sqrtf(r);
    wi->x() = sqrt_r * cosf(theta);
    wi->y() = sqrt_r * sinf(theta);
    wi->z() = sqrtf(std::max(0.f, 1.f - wi->x()*wi->x() - wi->y()*wi->y()));
    // wi is always chosen to be in +z hemisphere on side of normal

    *cos_wi = CosTheta(*wi);
    *pdf_sig = 1.0f / M_PI;
    return STColor3f(1.f / M_PI);
}

float Y0Lambertian::p_sig(const STVector3& wo, const STVector3& wi) const {
    if (wi.z() >= 0.f) {
        return 1.f / M_PI;
    }
    return 0.f;
}

std::string Y0Lambertian::getDescriptionString() const {
    return "Y0Lambertian";
}


STColor3f SpecularDiel::f(const STVector3& wo, const STVector3& wi) const {
    return STColor3f(0.f);
}

STColor3f SpecularDiel::sample_f(const STVector3& wo, STVector3* wi, float *pdf_sig, float* cos_wi) const {

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
    if (sint2 >= 1.) {
        *pdf_sig = 1.f;
        *wi = STVector3(-wo.x(), -wo.y(), wo.z());
        *cos_wi = AbsCosTheta(*wi);
        return R;
    } else {
        // choose either to reflect or transmit with probability proprotional to Fr
        float cost = sqrtf(std::max(0.f, 1.f - sint2));
        STColor3f Fr = FrDiel(AbsCosTheta(wo), cost, STColor3f(ei), STColor3f(et));

        float q = (Fr.r + Fr.g + Fr.b) / 3.f;
        if (randFloat() < q) {
            *pdf_sig = q;
            *wi = STVector3(-wo.x(), -wo.y(), wo.z());
            *cos_wi = AbsCosTheta(*wi);
            return Fr * R;
        } else {
            *pdf_sig = 1.f - q;
            if (entering) cost = -cost;
            float sintOverSini = eta;
            *wi = STVector3(sintOverSini * -wo.x(), sintOverSini * -wo.y(), cost);
            *cos_wi = AbsCosTheta(*wi);
            return (STColor3f(1.f) - Fr) * T;
        }
    }
}

float SpecularDiel::p_sig(const STVector3& wo, const STVector3& wi) const {
    return 0.f;
}

std::string SpecularDiel::getDescriptionString() const {
    std::stringstream ss;
    ss << "SpecularDiel  " << R.r << " " << R.g << " " << R.b
        << " " << T.r << " " << T.g << " " << T.b
        << " " << etai << " " << etat;
    return ss.str();
}



STColor3f SpecularCond::f(const STVector3& wo, const STVector3& wi) const {
    return STColor3f(0.f);
}

STColor3f SpecularCond::sample_f(const STVector3& wo, STVector3* wi, float *pdf_sig, float* cos_wi) const {
    // wi will always be wo reflected
    wi->x() = -wo.x();
    wi->y() = -wo.y();
    wi->z() = wo.z();
    *pdf_sig = 1.0f;     // 1*dirac

    *cos_wi = AbsCosTheta(*wi);

    // bsdf is Fr * delta(w-wi)
    if (wo.z() > 0.f) {
        return FrCond(CosTheta(wo), eta, k) * R;
    }
    return STColor3f(0.f);
}

float SpecularCond::p_sig(const STVector3& wo, const STVector3& wi) const {
    return 0.f;
}

std::string SpecularCond::getDescriptionString() const {
    std::stringstream ss;
    ss << "SpecularCond  " << R.r << " " << R.g << " " << R.b
        << " " << eta.r << " " << eta.g << " " << eta.b
        << " " << k.r << " " << k.g << " " << k.b;
    return ss.str();
}


STColor3f ScaledBsdf::f(const STVector3& wo, const STVector3& wi) const {
    return s * bsdf->f(wo, wi);
}

STColor3f ScaledBsdf::sample_f(const STVector3& wo, STVector3* wi, float *pdf_sig, float* cos_wi) const {
    return s * bsdf->sample_f(wo, wi, pdf_sig, cos_wi);
}

float ScaledBsdf::p_sig(const STVector3& wo, const STVector3& wi) const {
    return bsdf->p_sig(wo, wi);
}

std::string ScaledBsdf::getDescriptionString() const {
    std::stringstream ss;
    ss << "ScaledBsdf  " << bsdf->getDescriptionString();
    return ss.str();
}


Bsdf* newCopyBsdf(const Bsdf* bsdf) {
    switch (bsdf->type) {
    case Bsdf::Type::L: {
        const Lambertian* L_bsdf = dynamic_cast<const Lambertian*>(bsdf);
        return new Lambertian(*L_bsdf);
    }
    case Bsdf::Type::SC: {
        const SpecularCond* SC_bsdf = dynamic_cast<const SpecularCond*>(bsdf);
        return new SpecularCond(*SC_bsdf);
    }
    case Bsdf::Type::SD: {
        const SpecularDiel* SD_bsdf = dynamic_cast<const SpecularDiel*>(bsdf);
        return new SpecularDiel(*SD_bsdf);
    }
    default: {
        return NULL;
    }
    }
}
