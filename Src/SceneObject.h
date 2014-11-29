//#####################################################################
// Stanford CS148 Ray Tracer Example Code
// Copyright 2012, Ben Mildenhall
//#####################################################################

#ifndef RayTracer_SceneObject_h
#define RayTracer_SceneObject_h

#include "Shape.h"
#include "Ray.h"
#include "Material.h"
#include "STTransform4.h"
#include "Bsdf.h"

class SceneObject {
public:
    SceneObject(Shape* _shape=NULL, const Material* _material=NULL, const STTransform4* _transform=NULL, const int _texture_index=-1) :
        shape(_shape),
        aabb(NULL),
        material(_material==NULL?Material():(*_material)),
        transform(_transform==NULL?STTransform4::Identity():(*_transform)),
        texture_index(_texture_index),
        name("scene_object"),
        isLight(false),
        bsdf(&grayLambertian),
        emittedPower(0.f)
    {
        tInverse = transform.Inverse();
        tInverseTranspose = tInverse.Transpose();
        if (shape) aabb=shape->getAABB();
        if (aabb) aabb->rescale(transform);
        if (shape) name=shape->name;
    }

    // updated object that uses bsdf.
    SceneObject(Shape* shape, const STTransform4& transform, const Bsdf* bsdf, STColor3f emittedPower=STColor3f(0.f)) :
        shape(shape),
        aabb(NULL),
        material(),
        transform(transform),
        texture_index(-1),
        name("scene_object"),
        isLight(emittedPower.maxComponent() > 0.f),
        bsdf(bsdf),
        emittedPower(emittedPower)
    {
        tInverse = transform.Inverse();
        tInverseTranspose = tInverse.Transpose();
        if (shape) aabb = shape->getAABB();
        if (aabb) aabb->rescale(transform);
        if (shape) name = shape->name;
    }

    ~SceneObject()
    {
        //if(shape!=NULL){delete shape;shape=NULL;} ////may cause memory leak here
        if(aabb!=NULL){delete aabb;aabb=NULL;}
    }

    virtual bool doesIntersect(const Ray& ray) 
    { return shape->doesIntersect(ray.transform(tInverse)); }
    
    virtual Intersection* getIntersect(const Ray& ray) 
    {
        Intersection *inter = shape->getIntersect(ray.transform(tInverse));
        if (!inter) return NULL;
        inter->point = transform * inter->point;
        inter->normal = tInverseTranspose * inter->normal;
        inter->normal.Normalize();
        return inter;
    }

    virtual Intersection* getIntersectionWithObject(const Ray& ray, /*result*/SceneObject*& intersected_object)
    {
        Intersection* inter = getIntersect(ray);
        if(inter){intersected_object=this;}
        return inter;
    }


    // Assuming power is emitted in perfecly diffuse manner evenly across
    // surface area, so Le doesn't depend on x or w.
    virtual STColor3f Le() const {
        return emittedPower / (shape->getSurfaceArea() * M_PI);
    }
    // positional component of Le (irrandiance)
    virtual STColor3f Le0() const {
        return emittedPower / shape->getSurfaceArea();
    }

    virtual float Pa() const {
        return 1.f / shape->getSurfaceArea();
    }

    // chooses point y0 on surface and calculates an IntersectionBsdf for it.
    // y1 can then be chosen by sampling the bsdf in y0_intersection (which
    // really represents Le_1(y0, w))
    virtual void sample_y0(STPoint3* y0, STVector3* y0_n, Bsdf const** bsdf,
        float* pdf_a_y0, STColor3f* Le0_y0) {

        // uniform-randomly choose a point y0 on surface, record its normal
        *y0 = shape->uniformSampleSurface(y0_n);

        // transform y0 and y0_n from object-space to world-space
        *y0 = transform * *y0;
        *y0_n = tInverseTranspose * *y0_n;
        y0_n->Normalize();

        *bsdf = &y0Lambertian;

        *pdf_a_y0 = 1.f / shape->getSurfaceArea();
        *Le0_y0 = Le0();
    }


    Shape* shape;
    AABB* aabb;
    Material material;
	int texture_index;
    STTransform4 transform, tInverse, tInverseTranspose;
    std::string name;

    bool isLight;           // if true, bsdf is ignored.  If false, emittedPower is ignored
    const Bsdf* bsdf;             // replaces material
    STColor3f emittedPower;

private:
    SceneObject(const SceneObject& copy)    ////shallow copy
        : shape(copy.shape), aabb(copy.aabb), material(copy.material), transform(copy.transform), tInverse(copy.tInverse), tInverseTranspose(copy.tInverseTranspose), name(copy.name)
    {}
};

#endif
