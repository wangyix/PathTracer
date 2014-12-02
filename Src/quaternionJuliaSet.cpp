//
//  quaternionJuliaSet.cpp
//  RayTracer
//
//  Created by Marianna Neubauer on 11/21/14.
//  Copyright (c) 2014 Tianye Lu. All rights reserved.
//

#include "quaternionJuliaSet.h"

#define DELTA (1e-5f)
#define ITERATIONS (10)
#define ESCAPE_COEFFICIENT (2)
#define RADIUS (2)

// begin code from "OpenCL RayTraced Quaternion Julia-Set Example" in Mac Developer Library
float4 qmult( float4 q1, float4 q2 )
{
	float4 r;
	float3 t;
	
	float3 q1yzw = float3(q1.y, q1.z, q1.w);
	float3 q2yzw = float3(q2.y, q2.z, q2.w);
	float3 c = cross( q1yzw, q2yzw );
	
	t = q2yzw * q1.x + q1yzw * q2.x + c;
	r.x = q1.x * q2.x - dot( q1yzw, q2yzw );
	//r.yzw = t.xyz;
	r.y = t.x; r.z = t.y; r.w = t.z;
	
	return r;
}

float4 qsqr( float4 q )
{
	float4 r;
	float3 t;
	
	float3 qyzw = float3(q.y, q.z, q.w);
	
	t     = qyzw * q.x * 2.0f;
	r.x   = q.x * q.x - dot( qyzw, qyzw );
	//r.yzw = t.xyz;
	r.y = t.x; r.z = t.y; r.w = t.z;
	
	return r;
}

float length4(float4 f)
{
	return sqrt(f.x*f.x + f.y*f.y + f.z*f.z + f.w*f.w);
}

STVector3
EstimateNormalQJulia(
					 STPoint3 p,
					 float4 c,
					 int iterations )
{
	float4 qp = float4( p.x, p.y, p.z, 0.0f );
	float4 gx1 = qp - float4( DELTA, 0.0f, 0.0f, 0.0f );
	float4 gx2 = qp + float4( DELTA, 0.0f, 0.0f, 0.0f );
	float4 gy1 = qp - float4( 0.0f, DELTA, 0.0f, 0.0f );
	float4 gy2 = qp + float4( 0.0f, DELTA, 0.0f, 0.0f );
	float4 gz1 = qp - float4( 0.0f, 0.0f, DELTA, 0.0f );
	float4 gz2 = qp + float4( 0.0f, 0.0f, DELTA, 0.0f );
	
	for ( int i = 0; i < iterations; i++ )
	{
		gx1 = qsqr( gx1 ) + c;
		gx2 = qsqr( gx2 ) + c;
		gy1 = qsqr( gy1 ) + c;
		gy2 = qsqr( gy2 ) + c;
		gz1 = qsqr( gz1 ) + c;
		gz2 = qsqr( gz2 ) + c;
	}
	
	float nx = length4(gx2) - length4(gx1);
	float ny = length4(gy2) - length4(gy1);
	float nz = length4(gz2) - length4(gz1);
	
	STVector3 normal = STVector3( nx, ny, nz );
	normal.Normalize();
	
	return normal;
}

float
IntersectSphere(
    STVector3 rO,
    STVector3 rD,
    float radius )
{
	float fB = 2.0f * STVector3::Dot( rO, rD );
	float fB2 = fB * fB;
	float fC = STVector3::Dot( rO, rO ) - radius;
	float fT = (fB2 - 4.0f * fC);
	if (fT <= 0.0f)
		return 0.0f;
	float fD = sqrt( fT ); //half_sqrt is fast sqrt algorithm
	float fT0 = ( -fB + fD ) * 0.5f;
	float fT1 = ( -fB - fD ) * 0.5f;
	fT = fmin(fT0, fT1);
	return fT;
}

float4 reverse_raytrace(STVector3 rO, STVector3 rD, Ray ray, float4 mu, float epsilon);

float4
IntersectQJulia(
    STVector3 rO,
    STVector3 rD,
    float4 c,
    float epsilon,
    float escape)
{
	float rd = 0.0f;
	float dist = epsilon;
	while ( dist >= epsilon && rd < escape)
	{
		float4 z = float4( rO.x, rO.y, rO.z, 0.0f );
		float4 zp = float4( 1.0f, 0.0f, 0.0f, 0.0f );
		float zd = 0.0f;
		unsigned int count = 0;
		while(zd < escape && count < ITERATIONS)
		{
			zp = qmult(z, zp) * 2.0f;
			z = qsqr(z) + c;
			zd = dot(z, z);
			count++;
		}
		
		float normZ = length4( z );
		
		dist = 0.5f * normZ * log( normZ ) / length4( zp ); //was half_log which I think is a faster version of log
		
		rO = rO + rD * dist;
		rd = STVector3::Dot(rO, rO);
		
	}
	
	float4 hit; hit.x = rO.x; hit.y = rO.y; hit.z = rO.z; hit.w = dist;
	return hit;
}
// end of code from "OpenCL RayTraced Quaternion Julia-Set Example" in Mac Developer Library

float IntersectSphere2(STVector3 rO,
					   STVector3 rD,
						Ray ray)
{
	float a = rD.LengthSq();
	float b = 2 * STVector3::Dot(rD, rO);
	float c = (rO).LengthSq() - RADIUS * RADIUS;
	float disc = b * b - 4 * a * c;
	if (disc < 0.) return 0.0f;
	float t1 = (-b - sqrt(disc)) / (2 * a);
	float t2 = (-b + sqrt(disc)) / (2 * a);
	//return fmin(t1, t2);
	if (!(ray.inRange(t1) || ray.inRange(t2))) return 0.0f;
	return (ray.inRange(t1) ? t1 : t2);
}

Intersection* quaternionJuliaSet::getIntersect(const Ray &ray){
	
	STVector3 rD = ray.d;
	rD.Normalize();
	STVector3 rO = ray.e - center;
	float t = IntersectSphere2(rO, rD, ray);
	if (t<= 0) return NULL;
	
	/*STPoint3 point1 = ray.at(t);
	 STVector3 normal1 = point1 - center;
	 normal1.Normalize();
	 return new Intersection(t, point1, normal1);*/

	rO = rO + rD*t;
	
	// this code reverses the direction of the ray if it is moving out of the bouding sphere
	STPoint3 sphere_point;// = ray.at(t);
	sphere_point.x = rO.x; sphere_point.y = rO.y; sphere_point.z = rO.z;
	STVector3 sphere_normal = sphere_point - center;
	sphere_normal.Normalize();
	float cos_theta = STVector3::Dot(rD, sphere_normal);
	if (cos_theta > 0)
	{
		rD = -rD;
	}
	
	float escape_threshold = ESCAPE_COEFFICIENT*RADIUS;
	float4 hit = IntersectQJulia( rO, rD, mu, epsilon, escape_threshold);
	float dist = hit.w;
	if (dist >= epsilon) return NULL;
	
	
	STPoint3 point;
	point.x = hit.x; point.y = hit.y; point.z = hit.z;
	STVector3 normal = EstimateNormalQJulia( point, mu, ITERATIONS);
	
	t = (STVector3 (ray.e - point)).Length()/ray.d.Length(); //Hopefully this is the right t
	
	return new Intersection(t, point, normal);
	
}

bool quaternionJuliaSet::doesIntersect(const Ray& ray)
{
	STVector3 rD = ray.d;
	rD.Normalize();
	STVector3 rO = ray.e - center;
	float t = IntersectSphere2(rO, rD,ray);
	if (t<= 0) return NULL;
	
	rO = rO + rD*t;
	float escape_threshold = ESCAPE_COEFFICIENT*RADIUS;
	float4 hit = IntersectQJulia( rO, rD, mu, epsilon, escape_threshold);
	float dist = hit.w;
	
	if (dist >= epsilon) return false;
	else return true;
}

AABB* quaternionJuliaSet::getAABB()
{
	float radius = RADIUS;
	return new AABB(center.x - radius, center.x + radius, center.y - radius, center.y + radius, center.z - radius, center.z + radius);
}
