#ifndef RayTracer_QuadAA_h
#define RayTracer_QuadAA_h

#include "Shape.h"

class QuadAA : public Shape {
public:
    QuadAA(int uIdx, bool posNormal, float xmin, float xmax, float ymin, float ymax, float zmin, float zmax);

    bool getIntersect(const Ray& ray, Intersection* intersection) const override;
    bool doesIntersect(const Ray& ray) const override;
    void getAABB(const STTransform4& transform, AABB* aabb) const override;
    float getSurfaceArea() const override;
    STPoint3 uniformSampleSurface(STVector3* normal) const override;

private:
    void initializeUvwXyzArrays();
    void vector3ToUvw(const STVector3& v, float* uvw) const;
    void point3ToUvw(const STPoint3& v, float* uvw) const;

    // uIdx is the index indicating which coordinate out of x,y,z is constant for this quad
    // u,v,w is a permutation of x,y,z such that u is constant for this quad
    // For example, of y is constant for this quad, then uIdex=1, and u,v,w refer to y,z,x respectively.
    // Many calculations will convert x,y,z to u,v,w order first before computing something, and then
    // converting back before returning the final result.
    int uIdx;
    float u, vmin, vmax, wmin, wmax;
    float uNormalDir;    // 1.0 or -1.0 for +u or -u normal direction respectively

    //int uvw2xyz[3];     // helps convert between xyz and uvw index
    int xyz2uvw[3];
};

#endif
