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
    float x, y, z;

public:
    static STVector3 Zero();
    //
    // Initialization
    //
    inline STVector3();
    inline STVector3(const STVector3& v);
    //inline explicit STVector3(const STPoint3& p);
    inline STVector3(float x, float y, float z);
    //inline explicit STVector3(float s);

    //
    // Assignment
    //
    inline STVector3& operator=(const STVector3& v);

    //
    // Overloaded operators
    //
    inline STVector3& operator*=(float right);
    inline STVector3& operator/=(float right);
    inline STVector3& operator+=(const STVector3& right);
    inline STVector3& operator-=(const STVector3& right);

    //
    // Normalization
    //
    inline void normalize();
    //inline void SetLength(float newLength);

    //
    // Math
    //
    //inline float Length() const;
    //inline float LengthSq() const;
    inline float norm() const;
    inline float squaredNorm() const;

    //
    // Validation
    //
    //inline bool Valid() const;

    //
    // Component accessors
    //
    /*inline float& Component(unsigned int index)
    {
        return ((float *)this)[index];
    }

    inline float Component(unsigned int index) const
    {
        return ((const float *)this)[index];
    }*/
    inline float& operator()(int index) {
        return ((float *)this)[index];
    }
    inline const float& operator()(int index) const {
        return ((float *)this)[index];
    }

    inline float& x() { return x; }
    inline const float& x() const { return x; }
    inline float& y() { return y; }
    inline const float& y() const { return y; }
    inline float& z() { return z; }
    inline const float& z() const { return z; }

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
    /*inline static STVector3 Cross(const STVector3& left, const STVector3& right);
    inline static float Dot(const STVector3& left, const STVector3& right);
    inline static STVector3 DirectProduct(const STVector3& left, const STVector3& right);
    inline static STVector3 Lerp(const STVector3& left, const STVector3& right, float s);
    inline static STVector3 ComponentMax(const STVector3& left, const STVector3& right);
    inline static STVector3 ComponentMin(const STVector3& left, const STVector3& right);*/

    STVector3 cross3(const STVector3& b);
    float dot(const STVector3& b);

    STVector3 cwiseMin(const STVector3& p);
    STVector3 cwiseMax(const STVector3& p);
};

inline STVector3 operator*(const STVector3& left, float right);
inline STVector3 operator*(float left, const STVector3& right);
inline STVector3 operator/(const STVector3& left, float right);
inline STVector3 operator+(const STVector3& left, const STVector3& right);
inline STVector3 operator-(const STVector3& left, const STVector3& right);
inline STVector3 operator-(const STVector3& v);

inline STVector3 operator-(const STPoint3& left, const STPoint3& right);

//#include "STVector3.inl"

#endif  // __STVECTOR3_H__

#endif
