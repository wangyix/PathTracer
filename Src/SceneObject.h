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

    // non light-source object (bsdf is used)
    SceneObject(Shape* shape, const STTransform4& transform, const Bsdf* bsdf) :
        shape(shape),
        aabb(NULL),
        material(),
        transform(transform),
        texture_index(-1),
        name("scene_object"),
        isLight(false),
        bsdf(bsdf),
        emittedPower(0.f)
    {
        tInverse = transform.Inverse();
        tInverseTranspose = tInverse.Transpose();
        if (shape) aabb = shape->getAABB();
        if (aabb) aabb->rescale(transform);
        if (shape) name = shape->name;
    }

    // light-source object (power is emitted diffusely and evenly across surface)
    SceneObject(Shape* shape, const STTransform4& transform, STColor3f emittedPower) :
        shape(shape),
        aabb(NULL),
        material(),
        transform(transform),
        texture_index(-1),
        name("scene_object"),
        isLight(true),
        bsdf(&blackLambertian),
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
    // positional component of Le
    virtual STColor3f Le0() const {
        return emittedPower / shape->getSurfaceArea();
    }

    // chooses point y0 on surface and calculates an IntersectionBsdf for it.
    // y1 can then be chosen by sampling the bsdf in y0_intersection (which
    // really represents Le_1(y0, w))
    virtual void sample_y0(InterSectionBsdf* y0_intersection,
        float* pdf_a_y0, STColor3f* Le0_y0) {

        // uniform-randomly choose a point y0 on surface, record its normal
        STPoint3 y0;
        STVector3 y0_n;
        y0 = shape->uniformSampleSurface(&y0_n);

        // transform y0 and y0_n from object-space to world-space
        STPoint3 y0_w = transform * y0;
        STVector3 y0_n_w = tInverseTranspose * y0_n;
        y0_n_w.Normalize();

        // build IntersectionBsdf from y0_w and y0_n_w
        y0_intersection->setIntersection(Intersection(0.f, y0_w, y0_n_w));
        y0_intersection->setBsdf(&y0Lambertian);

        *pdf_a_y0 = 1.f / shape->getSurfaceArea();
        *Le0_y0 = Le0();
    }


    /*// chooses point y0 on surface and outgoing direction w.
    // also calculates Pa(y0), Le0(y0), Psig(y0, w), Le1(y0, w)
    virtual void sample_y0y1(STPoint3* y0, float* pdf_a_y0, STColor3f* Le0_y0,
        STVector3* w, float* pdf_sig_w, STColor3f* Le1_y0_w) {

        // uniform-randomly choose a point y0 on surface, record its normal
        STVector3 y0_n;
        *y0 = shape->uniformSampleSurface(&y0_n);

        *pdf_a_y0 = 1.f / shape->getSurfaceArea();
        *Le0_y0 = Le0();

        // construct unit axes around y0_n: I, J, K where K=y0_n
        STVector3 I, J;
        STVector3 K = y0_n;
        if (K.x == 0.f && K.y == 0.f) {
            I = STVector3(1.f, 0.f, 0.f);
            J = STVector3(0.f, 1.f, 0.f);
        } else {
            I = STVector3::Cross(K, STVector3(0.f, 0.f, 1.f));
            I.Normalize();
            J = STVector3::Cross(K, I);
            J.Normalize();
        }

        // choose w (cosine-sample the hemisphere)
        float r = (float)rand() / RAND_MAX;                     // [0, 1]
        float theta = (float)rand() / RAND_MAX * 2.0f * M_PI;   // [0, 2pi]
        float sqrt_r = sqrtf(r);
        float x = sqrt_r * cosf(theta);
        float y = sqrt_r * sinf(theta);
        float z = sqrtf((std::max)(0.f, 1.f - x*x - y*y));
        *w = x*I + y*J + z*K;
        *pdf_sig_w = 1.0f / M_PI;
        *Le1_y0_w = STColor3f(1.0f / M_PI);
        
        // transform y0, w from object-space to world-space
        *y0 = transform * *y0;
        *w = tInverseTranspose * *w;
    }*/


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
