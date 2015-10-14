//#####################################################################
// Stanford CS148 Ray Tracer Example Code
// Copyright 2012, Ben Mildenhall
//#####################################################################

#include "Sphere.h"

/*Intersection* Sphere::getIntersect(const Ray &ray) {
	float a = ray.d.LengthSq();
	float b = 2 * STVector3::Dot(ray.d, ray.e - center);
	float c = (ray.e - center).LengthSq() - radius * radius;
	float disc = b * b - 4 * a * c;
	if (disc < 0.) return NULL;
	float t1 = (-b - sqrt(disc)) / (2 * a);
	float t2 = (-b + sqrt(disc)) / (2 * a);
	if (!(ray.inRange(t1) || ray.inRange(t2))) return NULL;
	float t = (ray.inRange(t1) ? t1 : t2);
	STPoint3 point = ray.at(t);
	STVector3 normal = point - center;
	normal.Normalize();
	return new Intersection(t, point, normal);
}

Intersection** Sphere::getIntersections(const Ray &ray) {
    Intersection ** result = new Intersection*[maxInt];
    for (int i = 0; i < maxInt; i++) result[i] = NULL;
    
    float a = ray.d.LengthSq();
    float b = 2 * STVector3::Dot(ray.d, ray.e - center);
    float c = (ray.e - center).LengthSq() - radius * radius;
    float disc = b * b - 4 * a * c;
    if (disc < 0.) return result;
    float t1 = (-b - sqrt(disc)) / (2 * a);
    float t2 = (-b + sqrt(disc)) / (2 * a);
    if (!(ray.inRange(t1) || ray.inRange(t2))) return result;
    
    STPoint3 point = ray.at(t1);
    STVector3 normal = point - center;
    normal.Normalize();
    Intersection *inter1 = new Intersection(t1, point, normal);
    
    point = ray.at(t2);
    normal = point - center;
    normal.Normalize();
    Intersection *inter2 = new Intersection(t2, point, normal);
    
    if (!ray.inRange(t1)) {
        delete inter1;
        result[0] = inter2;
        return result;
    }
    if (!ray.inRange(t2)) {
        delete inter2;
        result[0] = inter1;
        return result;
    }
    
    if (t1 <= t2) {
        result[0] = inter1;
        result[1] = inter2;
    } else if (t1 == t2) {
        result[0] = inter1;
        delete inter2;
    } else {
        result[0] = inter2;
        result[1] = inter1;
    }
    return result;
}

bool Sphere::doesIntersect(const Ray &ray) {
    float a = ray.d.LengthSq();
    float b = 2 * STVector3::Dot(ray.d, ray.e - center);
    float c = (ray.e - center).LengthSq() - radius * radius;
    float disc = b * b - 4 * a * c;
    if (disc < 0.) return false;
    float t1 = (b * b - sqrt(disc)) / (2 * a);
    float t2 = (b * b + sqrt(disc)) / (2 * a);
    return ray.inRange(t1) || ray.inRange(t2);
}

bool Sphere::isInsideClosed(const STPoint3 &pt) {
    return ((pt - center).LengthSq() <= radius * radius + .001);
}

bool Sphere::isInsideOpen(const STPoint3 &pt) {
    return ((pt - center).LengthSq() < radius * radius - .001);
}
*/

void Sphere::getAABB(const STTransform4& transform, AABB* aabb) const {
#if USE_EIGEN
    float scale = transform.block(0, 0, 3, 1).norm(); // assuming transform does not warp shape
#else
    float scale = transform.columnnMagnitude(0);  // assuming transform does not warp shape
#endif
    float r = scale * radius;
    STPoint3 c = transform * center;
    *aabb = AABB(c.x() - r, c.x() + r, c.y() - r, c.y() + r, c.z() - r, c.z() + r);
}


bool Sphere::getIntersect(const Ray& ray, Intersection* intersection) const {
    float a = ray.d.squaredNorm();
    float b = 2 * ray.d.dot(ray.e - center);
    float c = (ray.e - center).squaredNorm() - radius * radius;
    float disc = b * b - 4 * a * c;
    float t1, t2;
    if (disc <= 0.f) return false;    // ray misses bounding sphere (<= instead of < ensures z!=0)
    if (b > 0.f) {
        float z = 0.5f * (-b - sqrtf(disc));
        t1 = z / a;  
        t2 = c / z;
    } else {
        float z = 0.5f * (-b + sqrtf(disc));
        t1 = c / z;
        t2 = z / a;
    }
    if (t2 < ray.t_min || t1 > ray.t_max) return false;
    // (!(ray.inRange(t1) || ray.inRange(t2))) return false;
    float t = (t1 >= ray.t_min ? t1 : t2);
    intersection->t = t;
    intersection->point = ray.at(t);
    intersection->normal = (intersection->point - center);
    intersection->normal.normalize();
    return true;
}

bool Sphere::doesIntersect(const Ray& ray) const {
    float a = ray.d.squaredNorm();
    float b = 2 * ray.d.dot(ray.e - center);
    float c = (ray.e - center).squaredNorm() - radius * radius;
    float disc = b * b - 4 * a * c;
    float t1, t2;
    if (disc <= 0.f) return false;    // ray misses bounding sphere (<= instead of < ensures z!=0)
    if (b > 0.f) {
        float z = 0.5f * (-b - sqrtf(disc));
        t1 = z / a;
        t2 = c / z;
    } else {
        float z = 0.5f * (-b + sqrtf(disc));
        t1 = c / z;
        t2 = z / a;
    }
    return (t2 >= ray.t_min && t1 <= ray.t_max);
}

float Sphere::getSurfaceArea() const {
    return 4.f * M_PI * radius * radius;
}

STPoint3 Sphere::uniformSampleSurface(STVector3* normal) const {
    // theta = 2*pi*u, phi = acos(2v-1)  where u,v uniform randoms in [0, 1]
    float u = randFloat();
    float v = randFloat();
    float theta = 2.f * M_PI * u;
    float phi = acosf(2.f*v - 1.f);

    normal->x() = sinf(phi) * cosf(theta);
    normal->y() = sinf(phi) * sinf(theta);
    normal->z() = cosf(phi);
    //normal->Normalize();  // SceneObject will normalize this
    return center + radius * *normal;
}
