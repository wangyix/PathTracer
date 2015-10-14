//
//  quaternionJuliaSet.cpp
//  RayTracer
//
//  Created by Marianna Neubauer on 11/21/14.
//  Copyright (c) 2014 Tianye Lu. All rights reserved.
//

#include "quaternionJuliaSet.h"
#include "SceneObject.h"
#include "STVector3.h"
#include "STPoint3.h"

#define DELTA (1e-5f)
#define ITERATIONS (12)
#define ESCAPE_COEFFICIENT (2)
#define RADIUS (1.5f)

// begin code from "OpenCL RayTraced Quaternion Julia-Set Example" in Mac Developer Library
float4 qmult(const float4& q1, const float4& q2)
{
	float4 r;
	float3 t;
	
	float3 q1yzw = float3(q1.y(), q1.z(), q1.w());
	float3 q2yzw = float3(q2.y(), q2.z(), q2.w());
    float3 c = q1yzw.cross(q2yzw);  //cross(q1yzw, q2yzw);
	
	t = q2yzw * q1.x() + q1yzw * q2.x() + c;
    r.x() = q1.x() * q2.x() - q1yzw.dot(q2yzw);   //dot(q1yzw, q2yzw);
	//r.yzw = t.xyz;
	r.y() = t.x(); r.z() = t.y(); r.w() = t.z();
	
	return r;
}

float4 qsqr(const float4& q)
{
	float4 r;
	float3 t;
	
	float3 qyzw = float3(q.y(), q.z(), q.w());
	
	t     = qyzw * q.x() * 2.0f;
    r.x() = q.x() * q.x() - qyzw.squaredNorm();   // dot(qyzw, qyzw);
	//r.yzw = t.xyz;
	r.y() = t.x(); r.z() = t.y(); r.w() = t.z();
	
	return r;
}

/*float length4(const float4& f)
{
	return sqrt(f.x()*f.x() + f.y()*f.y() + f.z()*f.z() + f.w()*f.w());
}*/

STVector3 EstimateNormalQJulia(const STPoint3& p, const float4& c, int iterations)
{
	float4 qp = float4( p.x(), p.y(), p.z(), 0.0f );
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
	
    float nx = gx2.norm() - gx1.norm(); // length4(gx2) - length4(gx1);
    float ny = gy2.norm() - gy1.norm(); // length4(gy2) - length4(gy1);
    float nz = gz2.norm() - gz1.norm(); // length4(gz2) - length4(gz1);
	
	STVector3 normal = STVector3( nx, ny, nz );
	normal.normalize();
	
	return normal;
}

/*float IntersectSphere(STVector3 rO, STVector3 rD, float radius)
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
}*/


float4 IntersectQJulia(const STVector3& _rO, const STVector3& rD, const float4& c, float epsilon, float escape)
{
    STVector3 rO = _rO;
	float rd = 0.0f;
	float dist = epsilon;
	while ( dist >= epsilon && rd < escape)
	{
		float4 z = float4( rO.x(), rO.y(), rO.z(), 0.0f );
		float4 zp = float4( 1.0f, 0.0f, 0.0f, 0.0f );
		float zd = 0.0f;
		unsigned int count = 0;
		while(zd < escape && count < ITERATIONS)
		{
			zp = qmult(z, zp) * 2.0f;
			z = qsqr(z) + c;
            zd = z.squaredNorm(); // dot(z, z);
			count++;
		}
		
        float normZ = z.norm();
		
		dist = 0.5f * normZ * log( normZ ) / zp.norm(); //was half_log which I think is a faster version of log
		
		rO = rO + rD * dist;
        rd = rO.squaredNorm();  // STVector3::Dot(rO, rO);
		
	}
	
	float4 hit; hit.x() = rO.x(); hit.y() = rO.y(); hit.z() = rO.z(); hit.w() = dist;
	return hit;
}
// end of code from "OpenCL RayTraced Quaternion Julia-Set Example" in Mac Developer Library




bool intersectBoundingSphere(const STPoint3& e, const STVector3& d_normalized, float* t1, float* t2) {
    STVector3 c_to_e = STVector3(e.x(), e.y(), e.z());
    
    // float a = 1.f;
    float b = 2.f * c_to_e.dot(d_normalized);   //STVector3::Dot(c_to_e, d_normalized);
    float c = c_to_e.squaredNorm() - RADIUS * RADIUS;
    float disc = b * b - 4.f  * c;    // a = 1
    if (disc <= 0.f) return false;    // ray misses bounding sphere (<= instead of < ensures z!=0)
    if (b > 0.f) {
        float z = 0.5f * (-b - sqrtf(disc));
        *t1 = z;        // z / a
        *t2 = c / z;
    } else {
        float z = 0.5f * (-b + sqrtf(disc));
        *t1 = c / z;
        *t2 = z;        // z / a
    }
    return true;
}


bool quaternionJuliaSet::getIntersect(const Ray& ray, Intersection* intersection) const {

    // check if this ray starts on the surface of this julia set
    bool rayLeavingSurface = false;
    bool rayEnteringSurface = false;
    //const float same_point_threshold = 0.0001f;
    if (ray.e_obj && ray.e_obj->getShape() == this) {
        // compare ray direction to the normal of the last intersection point to see if this ray
        // is entering or leaving the surface
        rayLeavingSurface = (ray.d_dot_e_normal >= 0.f);
        rayEnteringSurface = !rayLeavingSurface;
    }

    
    Ray ray_adj(ray);
    ray_adj.d.normalize();

    if (rayEnteringSurface) {
        // intersect this ray with the bounding sphere, and then reverse its direction
        // so it marches back into the bounding sphere to intersect with the julia set.
        // (for julia sets that are hollow, the intersection this returns probably won't be
        // where this refracted ray actually leaves the julia set, but oh well)
        float t1, t2;
        intersectBoundingSphere(ray_adj.e, ray_adj.d, &t1, &t2);
        ray_adj.e += t2 * ray_adj.d;
        ray_adj.d = -ray_adj.d;
    } else if (rayLeavingSurface) {
        // move the ray start forward by some epsilons to prevent intersecting the same spot
        ray_adj.e += 2.5f * epsilon * ray_adj.d;
    } else {
        // check if the ray starts outside the bounding sphere.  If so, move its start
        // position to where it first intersects the bounding sphere
        if (STVector3(ray_adj.e.x(), ray_adj.e.y(), ray_adj.e.z()).squaredNorm() > RADIUS * RADIUS) {
            float t1, t2;
            if (!intersectBoundingSphere(ray_adj.e, ray_adj.d, &t1, &t2)) {
                return false;            // ray does not intersect bounding sphere
            }
            ray_adj.e += t1 * ray_adj.d;
        }
    }


    // intersect the ray with the julia set
    float escape_threshold = ESCAPE_COEFFICIENT*RADIUS;
    float4 hit = IntersectQJulia(STVector3(ray_adj.e.x(), ray_adj.e.y(), ray_adj.e.z()),
        ray_adj.d, mu, epsilon, escape_threshold);
    float dist = hit.w();
    if (dist >= epsilon) {
        // ray-marched past the escape threshold instead of unbounding sphere radius
        // falling below epsilon; this ray does not intersect the julia set
        return false;
    }
    STPoint3 point = STPoint3(hit.x(), hit.y(), hit.z());
    
    // calculate t from the perspective of the original ray given to us
    float t = ray.d.dot(point - ray.e) / ray.d.squaredNorm();   // STVector3::Dot(point - ray.e, ray.d) / ray.d.LengthSq();
    if (!ray.inRange(t)) {
        return false;
    }

    // estimate the normal at the intersection with the julia set
    STVector3 normal = EstimateNormalQJulia(point, mu, ITERATIONS);

    intersection->t = t;
    intersection->point = point;
    intersection->normal = normal;
   
    return true;
}


bool quaternionJuliaSet::doesIntersect(const Ray& ray) const {

    Ray ray_adj(ray);
    float ray_d_length = ray.d.norm();
    ray_adj.d /= ray_d_length;  // ray_adj.d.Normalize()

    // check if the ray starts outside the bounding sphere.  If so, move its start
    // position to where it first intersects the bounding sphere
    if (STVector3(ray_adj.e.x(), ray_adj.e.y(), ray_adj.e.z()).squaredNorm() > RADIUS * RADIUS) {
        float t1, t2;
        if (!intersectBoundingSphere(ray_adj.e, ray_adj.d, &t1, &t2)) {
            return false;            // ray does not intersect bounding sphere
        }
        ray_adj.e += t1 * ray_adj.d;
    } else {
        // ray starts inside the bounding sphere.
        // intersect this ray with the bounding sphere, and then reverse its direction
        // so it marches back into the bounding sphere to intersect with the julia set.
        
        // Why not just let the ray start marching from where it is?  If the ray starts on the surface
        // of this julia set, then starting the march there causes weird things to happen sometimes (it may be
        // hard to distinguish between a close-by re-intersection or being marched back into the same intersection)
        // By doing a backwards march from the bounding sphere, we should avoid some of these problems.
        float t1, t2;
        intersectBoundingSphere(ray_adj.e, ray_adj.d, &t1, &t2);
        ray_adj.e += t2 * ray_adj.d;
        ray_adj.d = -ray_adj.d;
    }

    // intersect the ray with the julia set
    float escape_threshold = ESCAPE_COEFFICIENT*RADIUS;
    float4 hit = IntersectQJulia(STVector3(ray_adj.e.x(), ray_adj.e.y(), ray_adj.e.z()),
        ray_adj.d, mu, epsilon, escape_threshold);
    float dist = hit.w();
    if (dist >= epsilon) {
        // ray-marched past the escape threshold instead of unbounding sphere radius
        // falling below epsilon; this ray does not intersect the julia set
        return false;
    }
    STPoint3 point = STPoint3(hit.x(), hit.y(), hit.z());

    // calculate t from the perspective of the original ray given to us.  Pad the t_min with
    // some epsilons to prevent a ray starting on the julia set surface going outwards
    // intersecting at the same spot due to the backwards ray-march
    float ray_t_epsilon_pad = 2.5f * epsilon / ray_d_length;
    float t = ray.d.dot(point - ray.e) / (ray_d_length * ray_d_length);   // STVector3::Dot(point - ray.e, ray.d) / (ray_d_length * ray_d_length);

    return (t >= (ray.t_min + ray_t_epsilon_pad) && t <= ray.t_max);
}

void quaternionJuliaSet::getAABB(const STTransform4& transform, AABB* aabb) const {
#if USE_EIGEN
    float scale = transform.block(0, 0, 3, 1).norm(); // assuming transform does not warp shape
#else
    float scale = transform.columnnMagnitude(0);  // assuming transform does not warp shape
#endif
    float r = scale * RADIUS;
    STPoint3 c = transform * STPoint3(0.f, 0.f, 0.f);
    *aabb = AABB(c.x() - r, c.x() + r, c.y() - r, c.y() + r, c.z() - r, c.z() + r);
}


#if 0
Intersection* quaternionJuliaSet::getIntersect(const Ray &ray){
    float ray_d_length = ray.d.Length();

	STVector3 rD = ray.d;
    rD /= ray_d_length; //.Normalize();
	STVector3 rO (ray.e.x(), ray.e.y(), ray.e.z());
	float t = IntersectSphere2(rO, rD, ray);
	if (t<= 0) return NULL;
	
	/*STPoint3 point1 = ray.at(t);
	 STVector3 normal1 = point1 - center;
	 normal1.Normalize();
	 return new Intersection(t, point1, normal1);*/

	rO = rO + rD*t;
	
	// this code reverses the direction of the ray if it is moving out of the bouding sphere
	STVector3 sphere_point;// = ray.at(t);
	sphere_point.x() = rO.x(); sphere_point.y() = rO.y(); sphere_point.z() = rO.z();
    if (STVector3::Dot(rD, sphere_point) > 0)
	{
		rD = -rD;
	}
	
	float escape_threshold = ESCAPE_COEFFICIENT*RADIUS;
	float4 hit = IntersectQJulia( rO, rD, mu, epsilon, escape_threshold);
	float dist = hit.w();
    if (dist >= epsilon){
        // ray-marched past the escape threshold instead of unbounding sphere radius
        // falling below epsilon; this ray does not intersect the julia set
        return NULL;
    }
	
	
	STPoint3 point;
	point.x() = hit.x(); point.y() = hit.y(); point.z() = hit.z();
	STVector3 normal = EstimateNormalQJulia( point, mu, ITERATIONS);
	
	//t = (ray.e - point).Length()/ray.d.Length(); //Hopefully this is the right t

    // if ray intersected the bounding sphere from the inside, then the ray-march
    // back from that intersection may produce a hit point less than shadowBias
    // from the ray start or even slightly behind the ray start.
    
    // This comes up when checking for intersection with a ray that originates
    // on the surface of the julia set and heads outwards (i.e. reflected rays
    // off the surface of the julia set)

    // Since the hit point returned by IntersectQJulia() has an error proportional
    // to epsilon, we will only count this as an intersection if the hit point 
    // is more than 2 epsilons + shadowbias away from the ray start (assuming
    // ray.d is already normalized

    t = STVector3::Dot(point - ray.e, ray.d) / (ray_d_length*ray_d_length);//ray.d.LengthSq();
    if (t < ray.t_min + 2.f * epsilon / ray_d_length || t > ray.t_max) {//!ray.inRange(t)) {
        return NULL;
    }
	
	return new Intersection(t, point, normal);
	
}

bool quaternionJuliaSet::doesIntersect(const Ray& ray)
{
	STVector3 rD = ray.d;
	rD.Normalize();
    STVector3 rO(ray.e.x(), ray.e.y(), ray.e.z());
	float t = IntersectSphere2(rO, rD,ray);
	if (t<= 0) return NULL;
	
	rO = rO + rD*t;
	float escape_threshold = ESCAPE_COEFFICIENT*RADIUS;
	float4 hit = IntersectQJulia( rO, rD, mu, epsilon, escape_threshold);
	float dist = hit.w();
	
	if (dist >= epsilon) return false;
	else return true;
}
#endif
