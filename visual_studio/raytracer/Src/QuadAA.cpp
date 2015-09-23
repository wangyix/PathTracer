#include "QuadAA.h"
#include <assert.h>

QuadAA::QuadAA(int uIdx, bool uPosNormal, float xmin, float xmax, float ymin, float ymax, float zmin, float zmax)
    : uIdx(uIdx)
{
    assert(xmin <= xmax);
    assert(ymin <= ymax);
    assert(zmin <= zmax);

    name = "QuadAA";
    uNormalDir = uPosNormal ? 1.f : -1.f;

    initializeUvwXyzArrays();

    // set u and v,w min/max from x,y,z min/max
    float xyzMin[3] = { xmin, ymin, zmin };
    float xyzMax[3] = { xmax, ymax, zmax };
    float* uvwMin[3] = { &u, &vmin, &wmin };
    float* uvwmax[3] = { &u, &vmax, &wmax };
    for (int i = 0; i < 3; i++) {
        *uvwmax[xyz2uvw[i]] = xyzMax[i];
        *uvwMin[xyz2uvw[i]] = xyzMin[i];    
    }

    // u was set to the min for that dimension, if min/max are different for some reason
    if (xyzMin[uIdx] != xyzMax[uIdx]) {
        printf("WARNING: QuadAA min/max for u-dimension are different! Will use min for u.\n");
    }
}

void QuadAA::initializeUvwXyzArrays() {
    assert(0 <= uIdx && uIdx < 3);
    for (int i = 0; i < 3; i++) {
        xyz2uvw[i] = (i - uIdx + 3) % 3;
        //uvw2xyz[i] = (i + uIdx) % 3;
    }
}

void QuadAA::vector3ToUvw(const STVector3& v, float* uvw) const {
    for (int i = 0; i < 3; i++) {
        uvw[xyz2uvw[i]] = v(i);
    }
}
void QuadAA::point3ToUvw(const STPoint3& v, float* uvw) const {
    for (int i = 0; i < 3; i++) {
        uvw[xyz2uvw[i]] = v(i);
    }
}

bool QuadAA::getIntersect(const Ray& ray, Intersection* intersection) const {
    float e_uvw[3];
    float d_uvw[3];
    point3ToUvw(ray.e, e_uvw);
    vector3ToUvw(ray.d, d_uvw);
    const float &e_u = e_uvw[0], &e_v = e_uvw[1], &e_w = e_uvw[2];
    const float &d_u = d_uvw[0], &d_v = d_uvw[1], &d_w = d_uvw[2];
    
    if (d_u == 0.f) return false;   // ray is parallel with quad

    float p_uvw[3];     // intersection point
    float &p_u = p_uvw[0], &p_v = p_uvw[1], &p_w = p_uvw[2];

    float t = (u - e_u) / d_u;
    if (!ray.inRange(t)) return false;
    p_u = u;
    p_v = e_v + t * d_v;
    if (!(vmin <= p_v && p_v <= vmax)) return false;
    p_w = e_w + t * d_w;
    if (!(wmin <= p_w && p_w <= wmax)) return false;

    intersection->t = t;
    for (int i = 0; i < 3; i++) {
        intersection->point(i) = p_uvw[xyz2uvw[i]];
    }
    intersection->normal = STVector3(0.f, 0.f, 0.f);
    intersection->normal(uIdx) = uNormalDir;
    return true;
}

bool QuadAA::doesIntersect(const Ray& ray) const {
    float e_uvw[3];
    float d_uvw[3];
    point3ToUvw(ray.e, e_uvw);
    vector3ToUvw(ray.d, d_uvw);
    const float &e_u = e_uvw[0], &e_v = e_uvw[1], &e_w = e_uvw[2];
    const float &d_u = d_uvw[0], &d_v = d_uvw[1], &d_w = d_uvw[2];

    if (d_u == 0.f) return false;   // ray is parallel with quad

    float t = (u - e_u) / d_u;
    if (!ray.inRange(t)) return false;
    float p_v = e_v + t * d_v;
    if (!(vmin <= p_v && p_v <= vmax)) return false;
    float p_w = e_w + t * d_w;
    return (wmin <= p_w && p_w <= wmax);
}

void QuadAA::getAABB(const STTransform4& transform, AABB* aabb) const {
    float uvwMin[3] = { u, vmin, wmin };
    float uvwMax[3] = { u, vmax, wmax };
    float xyzMin[3];
    float xyzMax[3];
    for (int i = 0; i < 3; i++) {
        xyzMin[i] = uvwMin[xyz2uvw[i]];
        xyzMax[i] = uvwMax[xyz2uvw[i]];
    }
    *aabb = AABB(xyzMin[0], xyzMax[0], xyzMin[1], xyzMax[1], xyzMin[2], xyzMax[2]);
    aabb->rescale(transform);
}

float QuadAA::getSurfaceArea() const {
    return (vmax - vmin) * (wmax - wmin);
}

STPoint3 QuadAA::uniformSampleSurface(STVector3* normal) const {
    *normal = STVector3(0.f, 0.f, 0.f);
    normal->operator()(uIdx) = uNormalDir;

    float r1 = randFloat();
    float r2 = randFloat();
    float uvw[3] = { u,
                    (1 - r1)*vmin + r1*vmax,
                    (1 - r2)*wmin + r2*wmax };
    return STPoint3(uvw[xyz2uvw[0]], uvw[xyz2uvw[1]], uvw[xyz2uvw[2]]);
}

