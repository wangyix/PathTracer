//#####################################################################
// Stanford CS148 Ray Tracer Example Code
// Copyright 2013, Yue Chen
//#####################################################################

#include "AABB.h"

AABB::AABB(){}

AABB::AABB(float xmin, float xmax, float ymin, float ymax, float zmin, float zmax)
{
    this->xmin = xmin;
    this->xmax = xmax;
    this->ymin = ymin;
    this->ymax = ymax;
    this->zmin = zmin;
    this->zmax = zmax;
    this->xcenter = .5f*(xmin + xmax);
    this->ycenter = .5f*(ymin + ymax);
    this->zcenter = .5f*(zmin + zmax);
}

/*void AABB::rescale(const STTransform4& transform)
{
    std::vector<STPoint3> pts;
    pts.push_back(STPoint3(xmin, ymin, zmin));
    pts.push_back(STPoint3(xmin, ymin, zmax));
    pts.push_back(STPoint3(xmin, ymax, zmin));
    pts.push_back(STPoint3(xmin, ymax, zmax));
    pts.push_back(STPoint3(xmax, ymin, zmin));
    pts.push_back(STPoint3(xmax, ymin, zmax));
    pts.push_back(STPoint3(xmax, ymax, zmin));
    pts.push_back(STPoint3(xmax, ymax, zmax));
    pts[0] = transform * pts[0];
    pts[1] = transform * pts[1];
    pts[2] = transform * pts[2];
    pts[3] = transform * pts[3];
    pts[4] = transform * pts[4];
    pts[5] = transform * pts[5];
    pts[6] = transform * pts[6];
    pts[7] = transform * pts[7];
    xmin = xmax = pts[0].x();
    ymin = ymax = pts[0].y();
    zmin = zmax = pts[0].z();
    for(int i = 0;i < 8;++i)
    {
        if (pts[i].x() < xmin) xmin = pts[i].x();
        if (pts[i].x() > xmax) xmax = pts[i].x();
        if (pts[i].y() < ymin) ymin = pts[i].y();
        if (pts[i].y() > ymax) ymax = pts[i].y();
        if (pts[i].z() < zmin) zmin = pts[i].z();
        if (pts[i].z() > zmax) zmax = pts[i].z();
    }
    xcenter = .5f*(xmin + xmax);
    ycenter = .5f*(ymin + ymax);
    zcenter = .5f*(zmin + zmax);
}*/

static inline void intersectHelper(float* tin, bool* tinFound, float* tout, bool* toutFound,
                                   float eu, float ev, float ew, float du, float dv, float dw,
                                   float umin, float umax, float vmin, float vmax, float wmin, float wmax) {
    float uin, uout;
    if (du > 0.f) {
        uin = umin;
        uout = umax;
    } else if (du < 0.f) {
        uin = umax;
        uout = umin;
    } else {
        return;
    }

    if (!*tinFound) {
        float t = (uin - eu) / du;
        float v = ev + t * dv;
        if (vmin <= v && v <= vmax) {
            float w = ew + t * dw;
            if (wmin <= w && w <= wmax) {
                *tin = t;
                *tinFound = true;
            }
        }
    }
    if (!*toutFound) {
        float t = (uout - eu) / du;
        float v = ev + t * dv;
        if (vmin <= v && v <= vmax) {
            float w = ew + t * dw;
            if (wmin <= w && w <= wmax) {
                *tout = t;
                *toutFound = true;
            }
        }
    }
}

float AABB::intersect(const Ray& ray) const {
    bool tinFound = false, toutFound = false;
    float tin = -1.f, tout = -1.f;
    intersectHelper(&tin, &tinFound, &tout, &toutFound, ray.e.x(), ray.e.y(), ray.e.z(),
                    ray.d.x(), ray.d.y(), ray.d.z(), xmin, xmax, ymin, ymax, zmin, zmax);
    intersectHelper(&tin, &tinFound, &tout, &toutFound, ray.e.y(), ray.e.z(), ray.e.x(),
                    ray.d.y(), ray.d.z(), ray.d.x(), ymin, ymax, zmin, zmax, xmin, xmax);
    intersectHelper(&tin, &tinFound, &tout, &toutFound, ray.e.z(), ray.e.x(), ray.e.y(),
                    ray.d.z(), ray.d.x(), ray.d.y(), zmin, zmax, xmin, xmax, ymin, ymax);
    
    if (!tinFound || !toutFound) {
        // ray line misses AABB
        return -1.f;
    }
    if (ray.t_max < tin || ray.t_min > tout) {
        // ray segment misses AABB
        return -1.f;
    }
    return tin < ray.t_min ? tout : tin;
}

/* // Not used anywhere??
Intersection* AABB::getIntersect(const Ray& ray) const
{
	int face=0;float t=intersect(ray,face);if(t==-1.f)return NULL;
	STPoint3 point=ray.at(t);
	STVector3 normal=STVector3::Zero;normal.Component(face)=ray.d.Component(face)>=0.f?-1.f:1.f;
	normal.Normalize();
	return new Intersection(t,point,normal);
}*/

static inline bool doesIntersectHelper(float eu, float ev, float ew, float du, float dv, float dw,
                                       float umin, float umax, float vmin, float vmax, float wmin, float wmax) {
    if (du == 0.f)
        return false;
    {
        float t = (umin - eu) / du;
        float v = ev + t * dv;
        if (vmin <= v && v <= vmax) {
            float w = ew + t * dw;
            if (wmin <= w && w <= wmax) {
                return true;
            }
        }
    }
    {
        float t = (umax - eu) / du;
        float v = ev + t * dv;
        if (vmin <= v && v <= vmax) {
            float w = ew + t * dw;
            if (wmin <= w && w <= wmax) {
                return true;
            }
        }
    }
    return false;
}

bool AABB::doesIntersect(const Ray& ray) const {
    if (doesIntersectHelper(ray.e.x(), ray.e.y(), ray.e.z(),
        ray.d.x(), ray.d.y(), ray.d.z(), xmin, xmax, ymin, ymax, zmin, zmax)) {
        return true;
    }
    if (doesIntersectHelper(ray.e.y(), ray.e.z(), ray.e.x(),
        ray.d.y(), ray.d.z(), ray.d.x(), ymin, ymax, zmin, zmax, xmin, xmax)) {
        return true;
    }
    return doesIntersectHelper(ray.e.z(), ray.e.x(), ray.e.y(),
        ray.d.z(), ray.d.x(), ray.d.y(), zmin, zmax, xmin, xmax, ymin, ymax);
}

bool AABB::isInside(const STPoint3& point) const
{
	return point.x()>=xmin && point.x()<=xmax && point.y()>=ymin && point.y()<=ymax && point.z()>=zmin && point.z()<=zmax;
}

void AABB::combine(const AABB& b1,const AABB& b2, AABB* c)
{
    c->xmin = b1.xmin < b2.xmin ? b1.xmin : b2.xmin;
    c->xmax = b1.xmax > b2.xmax ? b1.xmax : b2.xmax;
    c->ymin = b1.ymin < b2.ymin ? b1.ymin : b2.ymin;
    c->ymax = b1.ymax > b2.ymax ? b1.ymax : b2.ymax;
    c->zmin = b1.zmin < b2.zmin ? b1.zmin : b2.zmin;
    c->zmax = b1.zmax > b2.zmax ? b1.zmax : b2.zmax;
    c->xcenter = 0.5f * (c->xmin + c->xmax);
    c->ycenter = 0.5f * (c->ymin + c->ymax);
    c->zcenter = 0.5f * (c->zmin + c->zmax);
}

AABB AABB::combine(const AABB& b1, const AABB& b2)
{
    AABB c;
    AABB::combine(b1, b2, &c);
    return c;
}

void AABB::combine(const STPoint3& p, AABB* c)
{
    if(p.x()<c->xmin)c->xmin=p.x();
    if(p.x()>c->xmax)c->xmax=p.x();
    if(p.y()<c->ymin)c->ymin=p.y();
    if(p.y()>c->ymax)c->ymax=p.y();
    if(p.z()<c->zmin)c->zmin=p.z();
    if(p.z()>c->zmax)c->zmax=p.z();
    c->xcenter = 0.5f * (c->xmin + c->xmax);
    c->ycenter = 0.5f * (c->ymin + c->ymax);
    c->zcenter = 0.5f * (c->zmin + c->zmax);
}
