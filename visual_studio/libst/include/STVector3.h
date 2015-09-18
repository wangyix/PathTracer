// STVector3.h
#ifndef __STVECTOR3_H__
#define __STVECTOR3_H__

#include "stForward.h"

#if USE_EIGEN

#define STVector3(x, y, z) Eigen::Vector4f(x, y, z, 0.f)

#else

#include <math.h>

/**
* STVector3 represents a 3-vector
*/
struct STVector3
{
private:
    float mx, my, mz;

public:
    static STVector3 Zero();
    //
    // Initialization
    //
    STVector3();
    //STVector3(const STVector3& v);
    //explicit STVector3(const STPoint3& p);
    STVector3(float x, float y, float z);
    //explicit STVector3(float s);

    //
    // Assignment
    //
    //STVector3& operator=(const STVector3& v);

    //
    // Overloaded operators
    //
    STVector3& operator*=(float right);
    STVector3& operator/=(float right);
    STVector3& operator+=(const STVector3& right);
    STVector3& operator-=(const STVector3& right);

    //
    // Normalization
    //
    void normalize();
    //void SetLength(float newLength);

    //
    // Math
    //
    //float Length() const;
    //float LengthSq() const;
    float norm() const;
    float squaredNorm() const;

    //
    // Validation
    //
    //bool Valid() const;

    //
    // Component accessors
    //
    /*float& Component(unsigned int index)
    {
        return ((float *)this)[index];
    }

    float Component(unsigned int index) const
    {
        return ((const float *)this)[index];
    }*/
    float& operator()(int index) {
        return ((float *)this)[index];
    }
    const float& operator()(int index) const {
        return ((float *)this)[index];
    }
    float& operator[](int index) {
        return ((float *)this)[index];
    }
    const float& operator[](int index) const {
        return ((float *)this)[index];
    }

    float& x() { return mx; }
    const float& x() const { return mx; }
    float& y() { return my; }
    const float& y() const { return my; }
    float& z() { return mz; }
    const float& z() const { return mz; }

    //
    // Constants
    //
    /*static const STVector3 Zero;
    static const STVector3 eX;
    static const STVector3 eY;
    static const STVector3 eZ;
    */

    //
    // Static math functions
    //
    /*static STVector3 Cross(const STVector3& left, const STVector3& right);
    static float Dot(const STVector3& left, const STVector3& right);
    static STVector3 DirectProduct(const STVector3& left, const STVector3& right);
    static STVector3 Lerp(const STVector3& left, const STVector3& right, float s);
    static STVector3 ComponentMax(const STVector3& left, const STVector3& right);
    static STVector3 ComponentMin(const STVector3& left, const STVector3& right);*/

    STVector3 cross3(const STVector3& b) const;
    float dot(const STVector3& b) const;

    STVector3 cwiseMin(const STVector3& p);
    STVector3 cwiseMax(const STVector3& p);

    bool isZero() const {
        return x() == 0.f && y() == 0.f && z() == 0.f;
    }
};

STVector3 operator*(const STVector3& left, float right);
STVector3 operator*(float left, const STVector3& right);
STVector3 operator/(const STVector3& left, float right);
STVector3 operator+(const STVector3& left, const STVector3& right);
STVector3 operator-(const STVector3& left, const STVector3& right);
STVector3 operator-(const STVector3& v);

STVector3 operator-(const STPoint3& left, const STPoint3& right);

//#include "STVector3.inl"

#endif  // __STVECTOR3_H__

#endif
