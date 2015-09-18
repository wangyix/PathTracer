// STPoint3.inl
#ifndef __STPOINT3_INL__
#define __STPOINT3_INL__

/**
* Inline file for STPoint3.h
*/

#include "STPoint3.h"

#if USE_EIGEN

#else

#include <algorithm>

#include "STVector3.h"

inline STPoint3::STPoint3()
{
    // Don't intialize; needs to match Eigen behavior
    /*x = 0;
    y = 0;
    z = 0;*/
}

inline STPoint3::STPoint3(float inX, float inY, float inZ)
{
    this->x = inX;
    this->y = inY;
    this->z = inZ;
}

/*inline STPoint3::STPoint3(const STVector3& v)
{
    x = v.x;
    y = v.y;
    z = v.z;
}*/

inline STPoint3& STPoint3::operator+=(const STVector3& right)
{
    this->x += right.x;
    this->y += right.y;
    this->z += right.z;
    return *this;
}

inline STPoint3& STPoint3::operator-=(const STVector3& right)
{
    this->x -= right.x;
    this->y -= right.y;
    this->z -= right.z;
    return *this;
}
/*
inline float STPoint3::Dist(const STPoint3& left, const STPoint3& right)
{
    return (right - left).Length();
}

inline float STPoint3::DistSq(const STPoint3& left, const STPoint3& right)
{
    return (right - left).LengthSq();
}
*/
inline STPoint3 operator+(const STPoint3& left, const STVector3& right)
{
    return STPoint3(left.x + right.x,
                    left.y + right.y,
                    left.z + right.z);
}

inline STPoint3 operator+(const STVector3& left, const STPoint3& right)
{
    return STPoint3(left.x + right.x,
                    left.y + right.y,
                    left.z + right.z);
}

inline STPoint3 operator-(const STPoint3& left, const STVector3& right)
{
    return STPoint3(left.x - right.x,
                    left.y - right.y,
                    left.z - right.z);
}

inline STPoint3 operator+(const STPoint3& left, const STPoint3& right)
{
    return STPoint3(left.x + right.x,
        left.y + right.y,
        left.z + right.z);
}

inline STPoint3 operator*(const STPoint3& left, const float& right)
{
    return STPoint3(left.x * right,
        left.y * right,
        left.z * right);
}

inline STPoint3 operator/(const STPoint3& left, const float& right)
{
    return STPoint3(left.x / right,
        left.y / right,
        left.z / right);
}

inline STPoint3 operator*(const float& left, const STPoint3& right) {
    return STPoint3(left * right.x,
        left * right.y,
        left * right.z);
}

/*
inline STPoint3 STPoint3::Max(const STPoint3& p1, const STPoint3& p2)
{
    return STPoint3((std::max)(p1.x,p2.x),
        (std::max)(p1.y,p2.y),
        (std::max)(p1.z,p2.z));
}

inline STPoint3 STPoint3::Min(const STPoint3& p1, const STPoint3& p2)
{
    return STPoint3((std::min)(p1.x,p2.x),
        (std::min)(p1.y,p2.y),
        (std::min)(p1.z,p2.z));
}*/

STPoint3 STPoint3::cwiseMin(const STPoint3& p) {
    return STPoint3((std::min)(this->x, p.x),
                    (std::min)(this->y, p.y),
                    (std::min)(this->z, p.z));
}

STPoint3 STPoint3::cwiseMax(const STPoint3& p) {
    return STPoint3((std::max)(this->x, p.x),
                    (std::max)(this->y, p.y),
                    (std::max)(this->z, p.z));
}

#endif  // __STPOINT3_INL__
#endif
