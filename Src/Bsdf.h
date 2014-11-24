#pragma once

#include "STColor3f.h"

inline float CosTheta(const STVector3 &w) { return w.z; }
inline float AbsCosTheta(const STVector3 &w) { return fabsf(w.z); }
inline float SinTheta2(const STVector3 &w) {
    return (std::max)(0.f, 1.f - CosTheta(w)*CosTheta(w));
}
inline float SinTheta(const STVector3 &w) {
    return sqrtf(SinTheta2(w));
}


// calculates reflectance Fr for Fresnel dielectrics and conductors.
// Dielectrics transmit 1-Fr, conductors absorb 1-Fr
STColor3f FrDiel(float cosi, float cost, const STColor3f& etai, const STColor3f& etat);
STColor3f FrCond(float cosi, const STColor3f& eta, const STColor3f& k);

STColor3f fresnelDielEvaluate(float cosi, float etai, float etat);

// pointed to by bsdf member of all SceneObjects that have no specified bsdf
static const Lambertian lambertianBsdf(STColor3f(0.5f));


class Bsdf {
public:
    virtual STColor3f f(const STVector3& wo, const STVector3& wi) const = 0;
    virtual STColor3f sample_f(const STVector3& wo, STVector3* wi, float *pdf_sig) const = 0;
};

class Lambertian : public Bsdf {
public:
    Lambertian(const STColor3f& R) : R(R) {};
    STColor3f f(const STVector3& wo, const STVector3& wi) const;
    STColor3f sample_f(const STVector3& wo, STVector3* wi, float *pdf_sig) const;
private:
    STColor3f R;    // albedo
};

class SpecularDiel : public Bsdf {
public:
    SpecularDiel(const STColor3f& R, const STColor3f& T, float etai, float etat) :
        R(R), T(T), etai(etai), etat(etat) {}
    STColor3f f(const STVector3& wo, const STVector3& wi) const;
    STColor3f sample_f(const STVector3& wo, STVector3* wi, float *pdf_sig) const;

private:
    STColor3f R;
    STColor3f T;
    float etai;
    float etat;
};

class SpecularCond : public Bsdf {
public:
    SpecularCond(const STColor3f& R, STColor3f& eta, STColor3f& k) :
        R(R), eta(eta), k(k) {}
    STColor3f f(const STVector3& wo, const STVector3& wi) const;
    STColor3f sample_f(const STVector3& wo, STVector3* wi, float *pdf_sig) const;

private:
    STColor3f R;
    STColor3f eta;
    STColor3f k;
};

class ScaledBsdf : public Bsdf {
public:
    ScaledBsdf(Bsdf *bsdf, const STColor3f& scale) : bsdf(bsdf), s(scale) {}
    STColor3f f(const STVector3& wo, const STVector3& wi) const;
    STColor3f sample_f(const STVector3& wo, STVector3* wi, float *pdf_sig) const;

private:
    Bsdf* bsdf;
    STColor3f s;
};

