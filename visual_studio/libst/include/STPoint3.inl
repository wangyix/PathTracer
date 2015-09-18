#if 0
// STPoint3.inl
#ifndef __STPOINT3_INL__
#define __STPOINT3_INL__

#include <algorithm>

#include "STVector3.h"

/**
* Inline file for STPoint3.h
*/

inline STPoint3::STPoint3()
{
    x = 0;
    y = 0;
    z = 0;
}

inline STPoint3::STPoint3(float inX, float inY, float inZ)
{
    x = inX;
    y = inY;
    z = inZ;
}

inline STPoint3::STPoint3(const STVector3& v)
{
    x = v.x;
    y = v.y;
    z = v.z;
}

inline STPoint3& STPoint3::operator+=(const STVector3& right)
{
    x += right.x;
    y += right.y;
    z += right.z;
    return *this;
}

inline STPoint3& STPoint3::operator-=(const STVector3& right)
{
    x -= right.x;
    y -= right.y;
    z -= right.z;
    return *this;
}

/**
* Returns distance between two points
*/
inline float STPoint3::Dist(const STPoint3& left, const STPoint3& right)
{
    return (right - left).Length();
}

/**
* Returns distance squared between two points
*/
inline float STPoint3::DistSq(const STPoint3& left, const STPoint3& right)
{
    return (right - left).LengthSq();
}

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
}

#endif  // __STPOINT3_INL__
#endif
