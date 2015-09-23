//#####################################################################
// Stanford CS148 Ray Tracer Example Code
// Copyright 2012, Ben Mildenhall
//#####################################################################

#ifndef RayTracer_SceneObjectTransform_h
#define RayTracer_SceneObjectTransform_h

#include <memory>
#include "SceneObject.h"
#include "Shape.h"
#include "Ray.h"
#include "STTransform4.h"
#include "Bsdf.h"
#include "Triangle.h"

class SceneObjectTransform : public SceneObject {
public:
#if USE_EIGEN
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif

    // updated object that uses bsdf.
    SceneObjectTransform(Shape* shape, const STTransform4& transform, const Bsdf* bsdf, STColor3f emittedPower = STColor3f(0.f)) :
        SceneObject(shape, bsdf, emittedPower),
        transform(transform),
        tInverse(transform.inverse()),
        tInverseTranspose(tInverse.transpose())
    {
#if USE_EIGEN
        // must do this for transposed transforms, otherwise the w of the resulting point/vector is wrong
        STTransform4& tInvTrans = *const_cast<STTransform4*>(&tInverseTranspose);
        tInvTrans(3, 0) = 0.f;
        tInvTrans(3, 1) = 0.f;
        tInvTrans(3, 2) = 0.f;

        scale = transform.block(0, 0, 3, 1).norm(); // assuming transform does not warp shape
#else
        scale = transform.columnnMagnitude(0);  // assuming transform does not warp shape
#endif
        if (shape) shape->getAABB(transform, &aabb);
        if (shape) name = shape->name;
    }

    /*~SceneObject()
    {
    //if(shape!=NULL){delete shape;shape=NULL;} ////may cause memory leak here
    if(aabb!=NULL){delete aabb;aabb=NULL;}
    if (bsdf) delete bsdf;
    }*/

    virtual bool doesIntersect(const Ray& ray) const override {
        return shape->doesIntersect(ray.transform(tInverse));
    }

    virtual bool getIntersect(const Ray& ray, Intersection* inter) const override {
        if (shape->getIntersect(ray.transform(tInverse), inter)) {
            inter->point = transform * inter->point;
            inter->normal = tInverseTranspose * inter->normal;
            inter->normal.normalize();
            return true;
        }
        return false;
    }

    virtual float getSurfaceArea() const override {
        return shape->getSurfaceArea() * scale * scale;
    }

    virtual STPoint3 uniformSampleSurface(STVector3* normal) const override {
        STPoint3 p = shape->uniformSampleSurface(normal);
        p = transform * p;
        *normal = tInverseTranspose * *normal;
        normal->normalize();
        return p;
    }

    /*
    Implemented in parent class SceneObjectNoTransform
    bool emitsLight() const { return isLight; }
    const STColor3f& getEmittedPower() const { return emittedPower; }
    const Bsdf* getBsdf() const { return bsdf.get(); }
    const Shape* getShape() const { return shape.get(); }
    const AABB& getAabb() const { return aabb; }*/

private:
    SceneObjectTransform(const SceneObjectTransform& copy)    ////shallow copy
        : SceneObject(copy),
        transform(copy.transform), tInverse(copy.tInverse),
        //tTranspose(copy.tTranspose),
        tInverseTranspose(copy.tInverseTranspose)
    {}

protected:
    const STTransform4 transform, tInverse, tInverseTranspose; //tTranspose, 
    float scale;        // cached, stores the scaling factor in transform (it's assumed that transform does not warp shape)
};



class TriangleMeshTriangle : public SceneObject {
public:
#if USE_EIGEN
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
    // this class is basically a triangle wrapped in a SceneObject interface, used exclusively
    // for TriangleMesh's AABBTree

    TriangleMeshTriangle(const Triangle& triangle)
        : SceneObject(NULL, NULL),    // shape and bsdf will be NULL
        triangle(triangle) {
        triangle.getAABB(STTransform4::Identity(), &aabb);
    }

    bool doesIntersect(const Ray& ray) const override{
        return triangle.doesIntersect(ray);
    }

    bool getIntersect(const Ray& ray, Intersection* inter)  const override{
        return triangle.getIntersect(ray, inter);
    }


    float getSurfaceArea() const override{
        return triangle.getSurfaceArea();
    }

    STPoint3 uniformSampleSurface(STVector3* normal) const override{
        return triangle.uniformSampleSurface(normal);
    }

private:
    Triangle triangle;
};


#endif
