//#####################################################################
// Stanford CS148 Ray Tracer Example Code
// Copyright 2012, Ben Mildenhall
//#####################################################################

#ifndef RayTracer_SceneObject_h
#define RayTracer_SceneObject_h

#include <memory>
#include "Shape.h"
#include "Ray.h"
#include "Material.h"
#include "STTransform4.h"
#include "Bsdf.h"

class SceneObject {
public:
    /*SceneObject(Shape* _shape=NULL, const Material* _material=NULL, const STTransform4* _transform=NULL, const int _texture_index=-1) :
        shape(_shape),
        aabb(NULL),
        material(_material==NULL?Material():(*_material)),
        transform(_transform==NULL?STTransform4::Identity():(*_transform)),
        texture_index(_texture_index),
        name("scene_object"),
        isLight(false),
        bsdf(new Lambertian(grayLambertian)),
        emittedPower(0.f)
    {
        tInverse = transform.Inverse();
        tInverseTranspose = tInverse.Transpose();
        if (shape) aabb=shape->getAABB();
        if (aabb) aabb->rescale(transform);
        if (shape) name=shape->name;
    }*/

    // updated object that uses bsdf.
    SceneObject(Shape* shape, const STTransform4& transform, const Bsdf* bsdf=NULL, STColor3f emittedPower=STColor3f(0.f)) :
        shape(shape),
        transform(transform),
        name("scene_object"),
        isLight(emittedPower.maxComponent() > 0.f),
        bsdf(bsdf ? bsdf : new Lambertian()),
        emittedPower(emittedPower)
    {
        tInverse = transform.Inverse();
        tInverseTranspose = tInverse.Transpose();
        scale = transform.columnnMagnitude(0);  // assuming transform does not warp shape
        if (shape) name = shape->name;
    }

    /*~SceneObject()
    {
        //if(shape!=NULL){delete shape;shape=NULL;} ////may cause memory leak here
        if(aabb!=NULL){delete aabb;aabb=NULL;}
        if (bsdf) delete bsdf;
    }*/

    virtual bool doesIntersect(const Ray& ray) const {
        return shape->doesIntersect(ray.transform(tInverse));
    }
    
    virtual bool getIntersect(const Ray& ray, Intersection* inter)  const {
        if (shape->getIntersect(ray.transform(tInverse), inter)) {
            inter->point = transform * inter->point;
            inter->normal = tInverseTranspose * inter->normal;
            inter->normal.Normalize();
            return true;
        }
        return false;
    }

    /*virtual Intersection* getIntersectionWithObject(const Ray& ray, SceneObject*& intersected_object) {
        Intersection* inter = getIntersect(ray);
        if (inter){ intersected_object = this; }
        return inter;
    }*/

    virtual float getSurfaceArea() const {
        return shape->getSurfaceArea() * scale * scale;
    }

    virtual STPoint3 uniformSampleSurface(STVector3* normal) const {
        STPoint3 p = shape->uniformSampleSurface(normal);
        p = transform * p;
        *normal = tInverseTranspose * *normal;
        normal->Normalize();
        return p;
    }


    bool emitsLight() const { return isLight; }
    const STColor3f& getEmittedPower() const { return emittedPower; }
    const Bsdf* getBsdf() const { return bsdf.get(); }

private:
    std::unique_ptr<const Shape> shape;
    //AABB* aabb;
    //Material material;
	//int texture_index;
    STTransform4 transform, tInverse, tInverseTranspose;
    float scale;        // cached, stores the scaling factor in transform (it's assumed that transform does not warp shape)
    std::string name;

    bool isLight;           // is true if emittedPower is non-zero
    std::unique_ptr<const Bsdf> bsdf;
    STColor3f emittedPower;

private:
    SceneObject(const SceneObject& copy)    ////shallow copy
        : /*shape(copy.shape), aabb(copy.aabb), material(copy.material),*/
        transform(copy.transform), tInverse(copy.tInverse), tInverseTranspose(copy.tInverseTranspose), name(copy.name)
    {}
};

#endif
