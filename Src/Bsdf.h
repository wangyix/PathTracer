#pragma once

#include "STColor3f.h"
#include"STVector3.h"

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

//STColor3f fresnelDielEvaluate(float cosi, float etai, float etat);



// convention: wi,wo both point outwards

class Bsdf {
public:
    enum Type{L, SC, SD};

public:
    virtual STColor3f f(const STVector3& wo, const STVector3& wi) const = 0;
    virtual STColor3f sample_f(const STVector3& wo, STVector3* wi, float *pdf_sig, float* cos_wi) const = 0;
    virtual float p_sig(const STVector3& wo, const STVector3& wi) const = 0;

    virtual bool isSpecular() const = 0;
    virtual bool worldSpaceParams() const { return false; }
public:
    Type type;
};



class Lambertian : public Bsdf {
public:
    Lambertian(const STColor3f& R) : R(R) { type = Type::L; }
    STColor3f f(const STVector3& wo, const STVector3& wi) const;
    STColor3f sample_f(const STVector3& wo, STVector3* wi, float *pdf_sig, float* cos_wi) const;
    float p_sig(const STVector3& wo, const STVector3& wi) const;
    bool isSpecular() const { return false; }
private:
    STColor3f R;    // albedo
};

// used for describing Le1(y0, w); direction wo is unused
// same as Lambertian except albedo is 1 and sample_f always gives wi in positive-z hemisphere
class Y0Lambertian : public Bsdf {
public:
    Y0Lambertian() { type = Type::L; }
    STColor3f f(const STVector3& wo, const STVector3& wi) const;
    STColor3f sample_f(const STVector3& wo, STVector3* wi, float *pdf_sig, float* cos_wi) const;
    float p_sig(const STVector3& wo, const STVector3& wi) const;
    bool isSpecular() const { return false; }
};

class SpecularDiel : public Bsdf {
public:
    SpecularDiel(const STColor3f& R, const STColor3f& T, float etai, float etat) :
        R(R), T(T), etai(etai), etat(etat) { type = Type::SD; }
    STColor3f f(const STVector3& wo, const STVector3& wi) const;
    STColor3f sample_f(const STVector3& wo, STVector3* wi, float *pdf_sig, float* cos_wi) const;
    float p_sig(const STVector3& wo, const STVector3& wi) const;
    bool isSpecular() const { return true; }

private:
    STColor3f R;
    STColor3f T;
    float etai;
    float etat;
};

class SpecularCond : public Bsdf {
public:
    SpecularCond(const STColor3f& R, STColor3f& eta, STColor3f& k) :
        R(R), eta(eta), k(k) { type = Type::SC; }
    STColor3f f(const STVector3& wo, const STVector3& wi) const;
    STColor3f sample_f(const STVector3& wo, STVector3* wi, float *pdf_sig, float* cos_wi) const;
    float p_sig(const STVector3& wo, const STVector3& wi) const;
    bool isSpecular() const { return true; }

private:
    STColor3f R;
    STColor3f eta;
    STColor3f k;
};

class ScaledBsdf : public Bsdf {
public:
    ScaledBsdf(const Bsdf *bsdf, const STColor3f& scale) : bsdf(bsdf), s(scale) { type = bsdf->type; }
    STColor3f f(const STVector3& wo, const STVector3& wi) const;
    STColor3f sample_f(const STVector3& wo, STVector3* wi, float *pdf_sig, float* cos_wi) const;
    float p_sig(const STVector3& wo, const STVector3& wi) const;
    bool isSpecular() const { return bsdf->isSpecular(); }
private:
    const Bsdf* bsdf;
    STColor3f s;
};


// used as the bsdf for SceneObjects that are constructed with the old constructor
static const Lambertian grayLambertian(STColor3f(0.5f));


// used as the bsdf for the IntersectionBsdf returned for the chosen vertex y0 
static const Y0Lambertian y0Lambertian;

Bsdf* newCopyBsdf(const Bsdf* bsdf);
