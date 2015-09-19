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
    float mx, my, mz;

public:
    STPoint3();
    STPoint3(float x, float y, float z);
    //explicit STPoint3(const STVector3& v);

    STPoint3& operator+=(const STVector3& right);
    STPoint3& operator-=(const STVector3& right);
	
    /*
    // Returns distance between two points
    // Called as STPoint3::Dist(left, right)
    static float Dist(const STPoint3& left, const STPoint3& right);

    // Returns distance squared between two points
    // Called as STPoint3::DistSq(left, right)
    static float DistSq(const STPoint3& left, const STPoint3& right);
    */
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

	friend std::ostream& operator<<(std::ostream& out,const STPoint3& p){out<<"["<<p.x()<<", "<<p.y()<<", "<<p.z()<<"]";return out;}

    //static STPoint3 Max(const STPoint3& p1, const STPoint3& p2);
    //static STPoint3 Min(const STPoint3& p1, const STPoint3& p2);
    STPoint3 cwiseMin(const STPoint3& p);
    STPoint3 cwiseMax(const STPoint3& p);
};

STPoint3 operator+(const STPoint3& left, const STVector3& right);
STPoint3 operator+(const STVector3& left, const STPoint3& right);
STPoint3 operator-(const STPoint3& left, const STVector3& right);

//STPoint3 operator+(const STPoint3& left, const STPoint3& right);
//STPoint3 operator*(const STPoint3& left, const float& right);
//STPoint3 operator/(const STPoint3& left, const float& right);

//STPoint3 operator*(const float& left, const STPoint3& right);

//#include "STPoint3.inl"

#endif  // __STPOINT3_H__

#endif
