// STVector3.inl
#ifndef __STVECTOR3_INL__
#define __STVECTOR3_INL__

/**
* Inline file for STVector3.h
*/

#include "STVector3.h"

#if USE_EIGEN

#else

#include <algorithm>

#include "STPoint3.h"
//#include "STUtil.h" // for STMin, STMax

inline STVector3::STVector3()
{
    // Don't intialize; needs to match Eigen behavior
    /*x = 0;
    y = 0;
    z = 0;*/
}

inline STVector3::STVector3(float inX, float inY, float inZ)
{
    this->x = inX;
    this->y = inY;
    this->z = inZ;
}

/*inline STVector3::STVector3(float s)
{
    x = s;
    y = s;
    z = s;
}*/

inline STVector3::STVector3(const STVector3 &v)
{
    this->x = v.x;
    this->y = v.y;
    this->z = v.z;
}

/*inline STVector3::STVector3(const STPoint3& p)
{
    x = p.x;
    y = p.y;
    z = p.z;
}*/

inline STVector3& STVector3::operator=(const STVector3 &v)
{
    this->x = v.x;
    this->y = v.y;
    this->z = v.z;
    return *this;
}

/**
* Length of vector
*/
inline float STVector3::norm() const
{
    return sqrtf(squaredNorm());
}

/**
* Length squared of vector
*/
inline float STVector3::squaredNorm() const
{
    return this->x * this->x + this->y * this->y + this->z * this->z;
}

/*// True if all elements are real values
inline bool STVector3::Valid() const
{
    // For standard floating-point math, the
    // "not-a-number" (NaN) representation
    // will test as not-equal to every value,
    // including itself!
    return ((x == x) && (y == y) && (z == z));
}*/

/**
* Sets the length of vector to 1
*/
inline void STVector3::normalize()
{
    float len = norm();
    if (len != 0.0f) {
        (*this) /= len;
    }
}

/*// Sets the length of vector to NewLength
inline void STVector3::SetLength(float newLength)
{
    float len = Length();
    if (len != 0.0f) {
        (*this) *= newLength / len;
    }
}*/

/*
inline STVector3 STVector3::Cross(
    const STVector3& left, const STVector3& right)
{
    return STVector3(left.y * right.z - left.z * right.y,
                     left.z * right.x - left.x * right.z,
                     left.x * right.y - left.y * right.x);
}

inline float STVector3::Dot(
    const STVector3& left, const STVector3& right)
{
    return left.x * right.x + left.y * right.y + left.z * right.z;
}

inline STVector3 STVector3::DirectProduct(
    const STVector3& left, const STVector3& right)
{
    return STVector3(left.x * right.x, left.y * right.y, left.z * right.z);
}

inline STVector3 STVector3::Lerp(
    const STVector3& left, const STVector3& right, float s)
{
    return left + s * (right - left);
}*/

STVector3 STVector3::cross3(const STVector3& b) {
    return STVector3(this->y * b.z - this->z * b.y,
                     this->z * b.x - this->x * b.z,
                     this->x * b.y - this->y * b.x);
}

float STVector3::dot(const STVector3& b) {
    return this->x*b.x + this->y*b.y + this->z*b.z;
}

/*
inline STVector3 STVector3::ComponentMax(
    const STVector3& left, const STVector3& right)
{
    return STVector3(
        STMax(left.x, right.x),
        STMax(left.y, right.y),
        STMax(left.z, right.z));
}

inline STVector3 STVector3::ComponentMin(
    const STVector3& left, const STVector3& right)
{
    return STVector3(
        STMin(left.x, right.x),
        STMin(left.y, right.y),
        STMin(left.z, right.z));
}
*/

STVector3 STVector3::cwiseMin(const STVector3& p) {
    return STVector3((std::min)(this->x, p.x),
        (std::min)(this->y, p.y),
        (std::min)(this->z, p.z));
}

STVector3 STVector3::cwiseMax(const STVector3& p) {
    return STVector3((std::max)(this->x, p.x),
        (std::max)(this->y, p.y),
        (std::max)(this->z, p.z));
}


inline STVector3 operator*(const STVector3& left, float right)
{
    STVector3 result(left);
    result *= right;
    return result;
}

inline STVector3 operator*(float left, const STVector3& right)
{
    STVector3 result(right);
    result *= left;
    return result;
}

inline STVector3 operator/(const STVector3& left, float right)
{
    STVector3 result(left);
    result /= right;
    return result;
}

inline STVector3 operator+(const STVector3& left, const STVector3& right)
{
    STVector3 result(left);
    result += right;
    return result;
}

inline STVector3 operator-(const STVector3& left, const STVector3& right)
{
    STVector3 result(left);
    result -= right;
    return result;
}

inline STVector3& STVector3::operator*=(float right)
{
    this->x *= right;
    this->y *= right;
    this->z *= right;
    return *this;
}

inline STVector3& STVector3::operator/=(float right)
{
    this->x /= right;
    this->y /= right;
    this->z /= right;
    return *this;
}

inline STVector3& STVector3::operator+=(const STVector3& right)
{
    this->x += right.x;
    this->y += right.y;
    this->z += right.z;
    return *this;
}

inline STVector3& STVector3::operator-=(const STVector3& right)
{
    this->x -= right.x;
    this->y -= right.y;
    this->z -= right.z;
    return *this;
}

inline STVector3 operator-(const STVector3& v)
{
    return STVector3(-v.x, -v.y, -v.z);
}

inline STVector3 operator-(const STPoint3& left, const STPoint3& right)
{
    return STVector3(left.x - right.x,
                     left.y - right.y,
                     left.z - right.z);
}

#endif  // __STVECTOR3_INL__
#endif
