//#####################################################################
// Stanford CS148 Ray Tracer Example Code
// Copyright 2012, Ben Mildenhall
//#####################################################################

#include "Cylinder.h"
#include <iostream>

using namespace std;

Intersection* Cylinder::getIntersect(const Ray &ray) {
    Intersection *interB = bottom->getIntersect(ray);
    Intersection *interT = top->getIntersect(ray);
    Intersection *flat = NULL;
    if (interT && interB) {
        if (isWithinCylinder(ray.at(interT->t))) {
            if (interB && isWithinCylinder(ray.at(interB->t)))
                flat = (interT->t <= interB->t ? interT : interB);
            else flat = interT;
        } else if (isWithinCylinder(ray.at(interB->t))) flat = interB;
    }
    else if (interT && isWithinCylinder(ray.at(interT->t))) flat = interT;
    else if (interB && isWithinCylinder(ray.at(interB->t))) flat = interB;
    if (flat != interT) delete interT;
    if (flat != interB) delete interB;
    
    STVector3 c = B - A;
    c.Normalize();
    STVector3 p = ray.e - A;
    float qa = ray.d.LengthSq() - pow(STVector3::Dot(c, ray.d), (int)2);
    float qb = STVector3::Dot(p, ray.d) * 2.f - STVector3::Dot(p, c) * STVector3::Dot(c, ray.d) * 2.f;
    float qc = p.LengthSq() - pow(STVector3::Dot(p, c), (int)2) - radius * radius;
    
    float disc = qb * qb - 4.f * qa * qc;
    if (disc < 0.) return flat;
    float t1 = (-qb - sqrt(disc)) / (2.f * qa);
    float t2 = (-qb + sqrt(disc)) / (2.f * qa);
    if (!(ray.inRange(t1) || ray.inRange(t2))) return flat;
    float t = (ray.inRange(t1) ? t1 : t2);
    STPoint3 point = ray.at(t);
    
    float frac = STVector3::Dot(point - A, c);
    if (frac < 0 || frac > (A - B).Length()) return flat;
    STVector3 normal = point - (A + frac * c);
    normal.Normalize();
    return closer(flat, new Intersection(t, point, normal)) ;
}

Intersection** Cylinder::getIntersections(const Ray &ray) {
    Intersection ** result = new Intersection*[maxInt];
    for (int i = 0; i < maxInt; i++) result[i] = NULL;
    
    STVector3 c = B - A;
    c.Normalize();
    STVector3 p = ray.e - A;
    float qa = ray.d.LengthSq() - pow(STVector3::Dot(c, ray.d), 2);
    float qb = STVector3::Dot(p, ray.d) * 2.f - STVector3::Dot(p, c) * STVector3::Dot(c, ray.d) * 2.f;
    float qc = p.LengthSq() - pow(STVector3::Dot(p, c), 2) - radius * radius;
    
    float disc = qb * qb - 4.f * qa * qc;
    if (disc < 0.) return result;
    float t1 = (-qb - sqrt(disc)) / (2.f * qa);
    float t2 = (-qb + sqrt(disc)) / (2.f * qa);
    if (!(ray.inRange(t1) || ray.inRange(t2))) return result;
        
    STPoint3 point = ray.at(t1);
    float frac = STVector3::Dot(point - A, c);
    STVector3 normal = point - (A + frac * c);
    normal.Normalize();
    Intersection *inter1 = new Intersection(t1, point, normal);
    
    point = ray.at(t2);
    frac = STVector3::Dot(point - A, c);
    normal = point - (A + frac * c);
    normal.Normalize();
    Intersection *inter2 = new Intersection(t2, point, normal);
    
    Intersection *interB = bottom->getIntersect(ray);
    Intersection *interT = top->getIntersect(ray);
    Intersection *flat = NULL;
    if (interT && interB) {
        if (isWithinCylinder(ray.at(interT->t))) {
            if (interB && isWithinCylinder(ray.at(interB->t))) {
                result[0] = (interT->t <= interB->t ? interT : interB);
                result[1] = (interT->t > interB->t ? interT : interB);
                return result;
            }
            else flat = interT;
        } else if (isWithinCylinder(ray.at(interB->t))) flat = interB;
    }
    else if (interT && isWithinCylinder(ray.at(interT->t))) flat = interT;
    else if (interB && isWithinCylinder(ray.at(interB->t))) flat = interB;
    
    
    if (!ray.inRange(t1)) {
        delete inter1;
        if (flat) {
            if (flat->t <= inter2->t) {
                result[0] = flat;
                result[1] = inter2;
            } else {
                result[1] = flat;
                result[0] = inter2;
            }
        }
        else result[0] = inter2;
        return result;
    }
    if (!ray.inRange(t2)) {
        delete inter2;
        if (flat) {
            if (flat->t <= inter1->t) {
                result[0] = flat;
                result[1] = inter1;
            } else {
                result[1] = flat;
                result[0] = inter1;
            }
        }
        else result[0] = inter1;
        return result;
    }
    if (flat) {
        if (isWithinCylinder(inter1->point)) {
            if (flat->t <= inter1->t) {
                result[0] = flat;
                result[1] = inter1;
                return result;
            } else {
                result[1] = flat;
                result[0] = inter1;
                return result;
            }
        } else if (isWithinCylinder(inter2->point)) {
            if (flat->t <= inter2->t) {
                result[0] = flat;
                result[1] = inter2;
                return result;
            } else {
                result[1] = flat;
                result[0] = inter2;
                return result;
            }
        }
    }
    
    if (!(isWithinCylinder(inter1->point) && isWithinCylinder(inter2->point))) return result;
    if (t1 <= t2) {
        result[0] = inter1;
        result[1] = inter2;
    } else if (t1 == t2) {
        result[0] = inter1;
    } else {
        result[0] = inter2;
        result[1] = inter1;
    }
    return result;
}

bool Cylinder::doesIntersect(const Ray &ray) {
    Intersection *interB = bottom->getIntersect(ray);
    Intersection *interT = top->getIntersect(ray);
    if (interT && isWithinCylinder(ray.at(interT->t))) return true;
    if (interB && isWithinCylinder(ray.at(interB->t))) return true;
    
    STVector3 c = B - A;
    c.Normalize();
    STVector3 p = ray.e - A;
    float qa = ray.d.LengthSq() - pow(STVector3::Dot(c, ray.d), 2);
    float qb = STVector3::Dot(p, ray.d) * 2 - STVector3::Dot(p, c) * STVector3::Dot(c, ray.d) * 2;
    float qc = p.LengthSq() - pow(STVector3::Dot(p, c), 2);
    
    float disc = qb * qb - 4 * qa * qc;
    if (disc < 0.) return false;
    float t1 = (-qb - sqrt(disc)) / (2 * qa);
    float t2 = (-qb + sqrt(disc)) / (2 * qa);
    if (!(ray.inRange(t1) || ray.inRange(t2))) return false;
    float t = (ray.inRange(t1) ? t1 : t2);
    STPoint3 point = ray.at(t);
    
    float frac = STVector3::Dot(point - A, c);
    
    if (frac < 0 || frac > (A - B).Length()) return false;
    
    return true;
}

Intersection* Cylinder::closer(Intersection * a, Intersection * b) {
    if (!a) return b;
    if (!b) return a;
    if (a->t <= b->t) {
        delete b;
        return a;
    } else {
        delete a;
        return b;
    }
}

bool Cylinder::isWithinCylinder(const STPoint3 &pt) {
    float frac = STVector3::Dot(pt - A, B - A) / (B - A).LengthSq();
    return frac >= -.001 && frac <= 1.001 && (pt - A - frac * (B - A)).LengthSq() <= radius * radius + .001;
    
}

bool Cylinder::isInsideOpen(const STPoint3& pt) {
    float frac = STVector3::Dot(pt - A, B - A) / (B - A).LengthSq();
    if (frac <= .001 || frac >= .999) return false;
    return (pt - A - frac * (B - A)).LengthSq() < radius * radius - .001;
}
bool Cylinder::isInsideClosed(const STPoint3& pt) {
    float frac = STVector3::Dot(pt - A, B - A) / (B - A).LengthSq();
    if (frac < -.001 || frac > 1.001) return false;
    return (pt - A - frac * (B - A)).LengthSq() <= radius * radius + .001;
}

AABB* Cylinder::getAABB()
{
	float xMin = min(A.x, B.x); 
	float xMax = max(A.x, B.x); 
	float yMin = min(A.y, B.y); 
	float yMax = max(A.y, B.y); 
	float zMin = min(A.z, B.z); 
	float zMax = max(A.z, B.z); 

	return new AABB(xMin-radius, xMax+radius, yMin-radius, yMax+radius, zMin-radius, zMax+radius);
}


float Cylinder::getSurfaceArea() const {
    return (2.0f * M_PI * radius * radius) + (2.0f * M_PI * radius * (A - B).Length());
}

STPoint3 Cylinder::uniformSampleSurface(STVector3* normal) const {
    // construct axes for cylinder (cylinder along z axis, from 0 to |A-B|
    // I, J are unit length, perpendicular to K, which is equal to B-A;
    STVector3 I, J;
    STVector3 K = B - A;
    if (K.x == 0.f && K.y == 0.f) {
        I = STVector3(1.f, 0.f, 0.f);
        J = STVector3(0.f, 1.f, 0.f);
    } else {
        I = STVector3::Cross(K, STVector3(0.f, 0.f, 1.f));
        I.Normalize();
        J = STVector3::Cross(K, I);
        J.Normalize();
    }

    // generate random in [0,total] and see if it's in [0,topCapCutoff],
    // [topCapCutoff,bottomCapCutoff], or [bottomCapCutoff,total] to choose either
    // the top cap, bottom cap, or side.
    float topCapCutoff = M_PI * radius * radius;
    float bottomCapCutoff = topCapCutoff + topCapCutoff;
    float total = bottomCapCutoff + (2.0f * M_PI * radius * (A - B).Length());

    float r = randFloat() * total;
    if (total < bottomCapCutoff) {
        // uniform-sample a unit disk
        float r = randFloat();                     // [0, 1]
        float theta = randFloat() * 2.0f * M_PI;   // [0, 2pi]
        float sqrt_r = sqrtf(r);
        float x = sqrt_r * cosf(theta);
        float y = sqrt_r * sinf(theta);

        if (total < topCapCutoff) {
            // get point on top cap
            *normal = K;
            normal->Normalize();
            return B + radius * (x*I + y*J);
        } else {
            // get point on bottom cap
            *normal = -K;
            normal->Normalize();
            return A + radius * (x*I + y*J);
        }
    } else {
        // sample the cylinder side (theta in [0,2pi], h in [0,1])
        float theta = randFloat() * 2.f * M_PI;
        float h = randFloat();

        *normal = cosf(theta)*I + sinf(theta)*J;
        normal->Normalize();
        return A + h*K + *normal*radius;
    }
}
