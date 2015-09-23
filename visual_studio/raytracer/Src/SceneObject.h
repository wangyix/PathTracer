#ifndef RayTracer_SceneObjectNoTransform_h
#define RayTracer_SceneObjectNoTransform_h

#include <memory>
#include "Shape.h"
#include "Ray.h"
#include "Bsdf.h"
#include "Triangle.h"

class SceneObject {
public:
    // updated object that uses bsdf.
    SceneObject(Shape* shape, const Bsdf* bsdf, STColor3f emittedPower = STColor3f(0.f)) :
        shape(shape),
        name("scene_object"),
        isLight(emittedPower.maxComponent() > 0.f),
        bsdf(bsdf),
        emittedPower(emittedPower)
    {
        if (shape) shape->getAABB(STTransform4::Identity(), &aabb);
        if (shape) name = shape->name;
    }

    virtual bool doesIntersect(const Ray& ray) const {
        return shape->doesIntersect(ray);
    }

    virtual bool getIntersect(const Ray& ray, Intersection* inter) const {
        return shape->getIntersect(ray, inter);
    }

    virtual float getSurfaceArea() const {
        return shape->getSurfaceArea();
    }

    virtual STPoint3 uniformSampleSurface(STVector3* normal) const {
        return shape->uniformSampleSurface(normal);
    }

    bool emitsLight() const { return isLight; }
    const STColor3f& getEmittedPower() const { return emittedPower; }
    const Bsdf* getBsdf() const { return bsdf.get(); }
    const Shape* getShape() const { return shape.get(); }
    const AABB& getAabb() const { return aabb; }

protected:
    SceneObject(const SceneObject& copy)    ////shallow copy
        : /*shape(copy.shape), aabb(copy.aabb), material(copy.material),*/
        name(copy.name)
    {}

protected:
    std::unique_ptr<const Shape> shape;
    AABB aabb;
    std::string name;

    bool isLight;           // is true if emittedPower is non-zero
    std::unique_ptr<const Bsdf> bsdf;
    STColor3f emittedPower;
};

#endif
