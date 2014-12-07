//
//  quaternionJuliaSet.h
//  RayTracer
//
//  Created by Marianna Neubauer on 11/21/14.
//  Copyright (c) 2014 Tianye Lu. All rights reserved.
//

#ifndef __RayTracer__quaternionJuliaSet__
#define __RayTracer__quaternionJuliaSet__

#include "Shape.h"


// begin from http://fastcpp.blogspot.com/p/common-datatypes.html
struct float4
{
	float4() {};
 	float4(float s) : x(s), y(s), z(s), w(s) {}
 	float4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
 	float x, y, z, w;
 
 	inline float4 operator*(float s) const { return float4(x*s, y*s, z*s, w*s); }
 	inline float4 operator+(const float4& a) const { return float4(x+a.x, y+a.y, z+a.z, w+a.w); }
	inline float4 operator-(const float4& a) const { return float4(x-a.x, y-a.y, z-a.z, w-a.w); }
};

// dot product of two float4 vectors
inline float dot(const float4& a, const float4& b) {
 	return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

struct float3
{
 	float3() {};
 	float3(float s) : x(s), y(s), z(s) {}
 	float3(float x, float y, float z) : x(x), y(y), z(z) {}
 	float x, y, z;
 
 	inline float3 operator*(float s) const { return float3(x*s, y*s, z*s); }
 	inline float3 operator+(const float3& a) const { return float3(x+a.x, y+a.y, z+a.z); }
	
};

// dot product of two float3 vectors
inline float dot(const float3& a, const float3& b) {
 	return a.x * b.x + a.y * b.y + a.z * b.z;
}
// end from http://fastcpp.blogspot.com/p/common-datatypes.html
/**
 * Returns cross product of two vectors
 */
inline float3 cross(const float3& left, const float3& right)
{
	return float3(left.y * right.z - left.z * right.y,
					 left.z * right.x - left.x * right.z,
					 left.x * right.y - left.y * right.x);
}


class quaternionJuliaSet : public Shape {
public:
	quaternionJuliaSet(float4 _mu, const float _epsilon)
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
	
	AABB* getAABB();*/

private:
	float4 mu;
	float epsilon;

    static Intersection lastIntersection;
};

#endif /* defined(__RayTracer__quaternionJuliaSet__) */
