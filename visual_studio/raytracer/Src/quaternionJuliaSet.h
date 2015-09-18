//
//  quaternionJuliaSet.h
//  RayTracer
//
//  Created by Marianna Neubauer on 11/21/14.
//  Copyright (c) 2014 Tianye Lu. All rights reserved.
//

#ifndef __RayTracer__quaternionJuliaSet__
#define __RayTracer__quaternionJuliaSet__

#include <unordered_map>
#include <thread>
#include <mutex>

#include "Shape.h"

#if USE_EIGEN

#include <Eigen/Dense>

typedef Eigen::Vector4f float4;
typedef Eigen::Vector3f float3;

#else

// begin from http://fastcpp.blogspot.com/p/common-datatypes.html
struct float4
{
private:
    float mx, my, mz, mw;
public:
	float4() {};
 	//float4(float s) : x(s), y(s), z(s), w(s) {}
 	float4(float x, float y, float z, float w) : mx(x), my(y), mz(z), mw(w) {}

    float& x() { return mx; }
    const float& x() const { return mx; }
    float& y() { return my; }
    const float& y() const { return my; }
    float& z() { return mz; }
    const float& z() const { return mz; }
    float& w() { return mw; }
    const float& w() const { return mw; }
 
    float4 operator*(float s) const { return float4(x()*s, y()*s, z()*s, w()*s); }
    float4 operator+(const float4& a) const { return float4(x() + a.x(), y() + a.y(), z() + a.z(), w() + a.w()); }
    float4 operator-(const float4& a) const { return float4(x() - a.x(), y() - a.y(), z() - a.z(), w() - a.w()); }

    float dot(const float4& b) const {
        return x() * b.x() + y() * b.y() + z() * b.z() + w() * b.w();
    }

    float squaredNorm() const {
        return x()*x() + y()*y() + z()*z() + w()*w();
    }
    float norm() const {
        return sqrtf(squaredNorm());
    }    
};

struct float3
{
private:
    float mx, my, mz;
public:
 	float3() {};
 	//float3(float s) : x(s), y(s), z(s) {}
 	float3(float x, float y, float z) : mx(x), my(y), mz(z) {}

    float& x() { return mx; }
    const float& x() const { return mx; }
    float& y() { return my; }
    const float& y() const { return my; }
    float& z() { return mz; }
    const float& z() const { return mz; }
 
    float3 operator*(float s) const { return float3(x()*s, y()*s, z()*s); }
    float3 operator+(const float3& a) const { return float3(x() + a.x(), y() + a.y(), z() + a.z()); }
	
    float dot(const float3& b) const {
        return x() * b.x() + y() * b.y() + z() * b.z();
    }
    
    float3 cross(const float3& right) const {
        return float3(y() * right.z() - z() * right.y(),
                      z() * right.x() - x() * right.z(),
                      x() * right.y() - y() * right.x());
    }

    float squaredNorm() const {
        return x()*x() + y()*y() + z()*z();
    }
    float norm() const {
        return sqrtf(squaredNorm());
    }
};

// end from http://fastcpp.blogspot.com/p/common-datatypes.html

#endif

class quaternionJuliaSet : public Shape {
public:
#if USE_EIGEN
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
	quaternionJuliaSet(const float4& _mu, const float _epsilon)
        : mu(_mu), epsilon(_epsilon)
	{
		this->name = "qjulia";
        //this->maxInt = 2;
	}

    bool getIntersect(const Ray& ray, Intersection* intersection) const override;
    bool doesIntersect(const Ray& ray) const override;
	
	/*Intersection* getIntersect(const Ray& ray);
	bool doesIntersect(const Ray& ray);
	
	Intersection** getIntersections(const Ray& ray) {return NULL;}
	bool isInsideOpen(const STPoint3& pt) { return false; }
	bool isInsideClosed(const STPoint3& pt) { return false; }
	*/
    void getAABB(const STTransform4& transform, AABB* aabb) const override;

private:
	float4 mu;
	float epsilon;
};

#endif /* defined(__RayTracer__quaternionJuliaSet__) */
