//#####################################################################
// Stanford CS148 Ray Tracer Example Code
// Copyright 2012, Ben Mildenhall
//#####################################################################

#include "Triangle.h"
#include "STVector3.h"

float determinant(const STVector3& a, const STVector3& b, const STVector3& c) {
    return a.x()*(b.y()*c.z()-b.z()*c.y())-b.x()*(a.y()*c.z()-a.z()*c.y())+c.x()*(a.y()*b.z()-a.z()*b.y());
}

/*Intersection* Triangle::getIntersect(const Ray &ray) {
    float detA = determinant(v1 - v2, v1 - v3, ray.d);
    if (detA == 0.) return NULL;
    float beta  = determinant(v1 - ray.e, v1 - v3, ray.d) / detA;
    float gamma = determinant(v1 - v2, v1 - ray.e, ray.d) / detA;
    float t     = determinant(v1 - v2, v1 - v3, v1 - ray.e) / detA;
    if (!ray.inRange(t) || beta < 0 || gamma < 0 || beta + gamma > 1) return NULL;
    STPoint3 point = ray.at(t);
	STVector3 normal = (1-beta-gamma)*n1+beta*n2+gamma*n3;
	STPoint2 uv = STPoint2((1-beta-gamma)*uv1.x()+beta*uv2.x()+gamma*uv3.x(),(1-beta-gamma)*uv1.y()+beta*uv2.y()+gamma*uv3.y());
    return new Intersection(t, point, normal, uv);
}

bool Triangle::doesIntersect(const Ray& ray) {
    float detA = determinant(v1 - v2, v1 - v3, ray.d);
    if (detA == 0.) return false;
    float beta  = determinant(v1 - ray.e, v1 - v3, ray.d) / detA;
    float gamma = determinant(v1 - v2, v1 - ray.e, ray.d) / detA;
    float t     = determinant(v1 - v2, v1 - v3, v1 - ray.e) / detA;
    return ray.inRange(t) && beta >= 0 && gamma >= 0 && beta + gamma <= 1;
}

Intersection** Triangle::getIntersections (const Ray & ray) {
    Intersection *inter = getIntersect(ray);
    Intersection ** result = &inter;
    return result;
}*/

void Triangle::getAABB(const STTransform4& transform, AABB* aabb) const {
    float xmin, xmax, ymin, ymax, zmin, zmax;

    STPoint3 vv1 = transform * v1;
    STPoint3 vv2 = transform * v2;
    STPoint3 vv3 = transform * v3;

    if(vv1.x() > vv2.x()){xmin = vv2.x();xmax = vv1.x();}
    else{xmin = vv1.x();xmax = vv2.x();}
    if(vv3.x() < xmin)xmin = vv3.x();
    if(vv3.x() > xmax)xmax = vv3.x();
    
    if(vv1.y() > vv2.y()){ymin = vv2.y();ymax = vv1.y();}
    else{ymin = vv1.y();ymax = vv2.y();}
    if(vv3.y() < ymin)ymin = vv3.y();
    if(vv3.y() > ymax)ymax = vv3.y();
    
    if(vv1.z() > vv2.z()){zmin = vv2.z();zmax = vv1.z();}
    else{zmin = vv1.z();zmax = vv2.z();}
    if(vv3.z() < zmin)zmin = vv3.z();
    if(vv3.z() > zmax)zmax = vv3.z();
    
    *aabb = AABB(xmin, xmax, ymin, ymax, zmin, zmax);
}


bool Triangle::getIntersect(const Ray& ray, Intersection* intersection) const {
    float detA = determinant(v1 - v2, v1 - v3, ray.d);
    if (detA == 0.) return false;
    float beta = determinant(v1 - ray.e, v1 - v3, ray.d) / detA;
    float gamma = determinant(v1 - v2, v1 - ray.e, ray.d) / detA;
    float t = determinant(v1 - v2, v1 - v3, v1 - ray.e) / detA;
    if (!ray.inRange(t) || beta < 0 || gamma < 0 || beta + gamma > 1) return false;
    intersection->t = t;
    intersection->point = ray.at(t);
    intersection->normal = (1 - beta - gamma)*n1 + beta*n2 + gamma*n3;  // sceneObject will normalize this later
    //intersection->uv = STPoint2((1 - beta - gamma)*uv1.x() + beta*uv2.x() + gamma*uv3.x(), (1 - beta - gamma)*uv1.y() + beta*uv2.y() + gamma*uv3.y());
    return true;
}

bool Triangle::doesIntersect(const Ray& ray) const {
    float detA = determinant(v1 - v2, v1 - v3, ray.d);
    if (detA == 0.) return false;
    float beta = determinant(v1 - ray.e, v1 - v3, ray.d) / detA;
    float gamma = determinant(v1 - v2, v1 - ray.e, ray.d) / detA;
    float t = determinant(v1 - v2, v1 - v3, v1 - ray.e) / detA;
    return (ray.inRange(t) && beta >= 0 && gamma >= 0 && beta + gamma <= 1);
}

float Triangle::getSurfaceArea() const {
    return 0.5f * (v2 - v1).cross3(v3 - v1).norm();
}

STPoint3 Triangle::uniformSampleSurface(STVector3* normal) const {
    // sample triangle: P=(1-sqrt(r1))A + sqrt(r1)(1-r2)B + sqrt(r1)r2C
    // where r1, r2 are uniform randoms in [0, 1]
    float r1 = randFloat();
    float r2 = randFloat();
    float sqrt_r1 = sqrtf(r1);

    // barycentric coordinates for chosen point
    float c1 = 1 - sqrt_r1;
    float c2 = sqrt_r1 * (1 - r2);
    float c3 = sqrt_r1 * r2;

    *normal = c1*n1 + c2*n2 + c3*n3;
    //normal->Normalize();  // SceneObject will normalize this
    STVector3 vv1 = STVector3(v1.x(), v1.y(), v1.z());
    STVector3 vv2 = STVector3(v2.x(), v2.y(), v2.z());
    STVector3 vv3 = STVector3(v3.x(), v3.y(), v3.z());
    STVector3 ret = c1*vv1 + c2*vv2 + c3*vv3;
    return STPoint3(ret.x(), ret.y(), ret.z());
}
