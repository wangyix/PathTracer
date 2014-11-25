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
        bsdf(&lambertianBsdf),
        emittedPower(0.f)
    {
        tInverse = transform.Inverse();
        tInverseTranspose = tInverse.Transpose();
        if (shape) aabb=shape->getAABB();
        if (aabb) aabb->rescale(transform);
        if (shape) name=shape->name;
    }

    // non light-source object (bsdf is used)
    SceneObject(Shape* shape, const STTransform4& transform, Bsdf* bsdf) :
        shape(shape),
        aabb(NULL),
        material(),
        transform(transform),
        texture_index(-1),
        name("scene_object"),
        bsdf(bsdf),
        emittedPower(0.0f)
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
        bsdf(&lambertianBsdf),
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


    // COMBINE sample_y0 and sample_y0y1 ????? 
    //  should sampled y0 and w be returned in world space???????? prolly


    // chooses point y0 on surface and outgoing direction w.
    // also calculates Pa(y0), Le0(y0), Psig(y0, w), Le1(y0, w)
    virtual void sample_y0y1(STPoint3* y0, float* pdf_a_y0, STColor3f* Le0_y0,
        STVector3* w, float* pdf_sig_w, STColor3f* Le1_y0_w) {

        // choose y0
        //*y0 = shape->uniformSamplePoint();
        *pdf_a_y0 = 1.f / shape->getSurfaceArea();
        *Le0_y0 = Le0();

        // choose w (cosine-sample the hemisphere)
        float r = (float)rand() / RAND_MAX;                     // [0, 1]
        float theta = (float)rand() / RAND_MAX * 2.0f * M_PI;   // [0, 2pi]
        float sqrt_r = sqrtf(r);
        w->x = sqrt_r * cosf(theta);
        w->y = sqrt_r * sinf(theta);
        w->z = sqrtf((std::max)(0.f, 1.f - w->x*w->x - w->y*w->y));
        *pdf_sig_w = 1.0f / M_PI;
        *Le1_y0_w = STColor3f(1.0f / M_PI);
        
        // transform y0, w from normal-space to object-space to world-space
    }



    /*// chooses a point y0 on surface, returns Le0(y0), also calculates Pa(y0) and the normal at y0.
    // y0, 
    virtual STColor3f sample_y0(STPoint3* y0, STVector3* y0_n, float* pdf_a) {
        //*y0 = shape->uniformSamplePoint();
        *pdf_a = 1.f / shape->getSurfaceArea();
        return Le0();
    }

    // chooses a ray direction w from y0, returns Le1(y0, w).
    // also calculates Psig(w)
    virtual STColor3f sample_y0y1(const STPoint3& y0, const STVector3 y0_n,
        STVector3* wo, float* pdf_sig) {
        
        // cosine-sample the hemisphere
        float r = (float)rand() / RAND_MAX;                     // [0, 1]
        float theta = (float)rand() / RAND_MAX * 2.0f * M_PI;   // [0, 2pi]
        float sqrt_r = sqrtf(r);
        STVector3 woN;
        woN.x = sqrt_r * cosf(theta);
        woN.y = sqrt_r * sinf(theta);
        woN.z = sqrtf((std::max)(0.f, 1.f - woN.x*woN.x - woN.y*woN.y));
        
        // transform from normal-space to object-space
        

        *pdf_sig = 1.0f / M_PI;
        return STColor3f(1.0f / M_PI);  // Le1(y0, w)
    }*/

    Shape* shape;
    AABB* aabb;
    Material material;
	int texture_index;
    STTransform4 transform, tInverse, tInverseTranspose;
    std::string name;

    Bsdf* bsdf;             // replaces material
    STColor3f emittedPower; // if != 0, then this is light source, and bsdf is ignored

private:
    SceneObject(const SceneObject& copy)    ////shallow copy
        : shape(copy.shape), aabb(copy.aabb), material(copy.material), transform(copy.transform), tInverse(copy.tInverse), tInverseTranspose(copy.tInverseTranspose), name(copy.name)
    {}
};

#endif
