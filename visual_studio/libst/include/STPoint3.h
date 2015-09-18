// STPoint3.h
#ifndef __STPOINT3_H__
#define __STPOINT3_H__

#include "stForward.h"

#if USE_EIGEN

#define STPoint3(x, y, z) Eigen::Vector4f(x, y, z, 1.f)

#else

#include <math.h>
#include <iostream>

/**
*  Simple struct to represent 3D points
*/
struct STPoint3
{
private:
    float x, y, z;

public:
    //static const STPoint3 Origin;
    static STPoint3 Zero();

    inline STPoint3();
    inline STPoint3(float x, float y, float z);
    //inline explicit STPoint3(const STVector3& v);

    inline STPoint3& operator+=(const STVector3& right);
    inline STPoint3& operator-=(const STVector3& right);
	
    /*
    // Returns distance between two points
    // Called as STPoint3::Dist(left, right)
    static inline float Dist(const STPoint3& left, const STPoint3& right);

    // Returns distance squared between two points
    // Called as STPoint3::DistSq(left, right)
    static inline float DistSq(const STPoint3& left, const STPoint3& right);
    */
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

	friend std::ostream& operator<<(std::ostream& out,const STPoint3& p){out<<"["<<p.x<<", "<<p.y<<", "<<p.z<<"]";return out;}

    //static STPoint3 Max(const STPoint3& p1, const STPoint3& p2);
    //static STPoint3 Min(const STPoint3& p1, const STPoint3& p2);
    STPoint3 cwiseMin(const STPoint3& p);
    STPoint3 cwiseMax(const STPoint3& p);
};

inline STPoint3 operator+(const STPoint3& left, const STVector3& right);
inline STPoint3 operator+(const STVector3& left, const STPoint3& right);
inline STPoint3 operator-(const STPoint3& left, const STVector3& right);

//inline STPoint3 operator+(const STPoint3& left, const STPoint3& right);
//inline STPoint3 operator*(const STPoint3& left, const float& right);
//inline STPoint3 operator/(const STPoint3& left, const float& right);

//inline STPoint3 operator*(const float& left, const STPoint3& right);

//#include "STPoint3.inl"

#endif  // __STPOINT3_H__

#endif
