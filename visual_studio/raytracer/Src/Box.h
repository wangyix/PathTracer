//#####################################################################
// Stanford CS148 Ray Tracer Example Code
// Copyright 2012, Ben Mildenhall
//#####################################################################

#ifndef RayTracer_Box_h
#define RayTracer_Box_h

#include "Shape.h"
#include "Triangle.h"

class Box : public Shape {
public:

#if USE_EIGEN
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif

    Box(const STPoint3& o, const STPoint3& x, const STPoint3& y, const STPoint3& z)
        : o(o), i(x - o), j(y - o), k(z - o)
    {
        this->name = "box";
        fillCob();
        initTriangles();
    }
    Box(const STPoint3& center,const STVector3& size)   ////axis aligned box
        : i(STVector3(size.x(), 0.f, 0.f)), j(STVector3(0.f, size.y(), 0.f)), k(STVector3(0.f, 0.f, size.z()))
    {
        o = center - size*.5f;
        this->name = "box";
        fillCob();
        initTriangles();
    }
    Box(const Box& copy)
    {
        cob=copy.cob;
        this->name=copy.name;
        for(int i=0;i<=12;i++){
            this->sides[i] = copy.sides[i];
        }
    }

    void getAABB(const STTransform4& transform, AABB* aabb) const override {
        *aabb = AABB(FLT_MAX, -FLT_MAX, FLT_MAX, -FLT_MAX, FLT_MAX, -FLT_MAX);
        AABB::combine(transform * o, aabb);
        AABB::combine(transform * (o + i), aabb);
        AABB::combine(transform * (o + j), aabb);
        AABB::combine(transform * (o + k), aabb);
        AABB::combine(transform * (o + i + j), aabb);
        AABB::combine(transform * (o + j + k), aabb);
        AABB::combine(transform * (o + k + i), aabb);
        AABB::combine(transform * (o + i + j + k), aabb);
    }

    bool getIntersect(const Ray& ray, Intersection* intersection) const override {
        Intersection min_int(FLT_MAX, STPoint3(0.f, 0.f, 0.f), STVector3(0.f, 0.f, 0.f));
        for (int i = 0; i < 12; i++) {
            Intersection inter;
            if (sides[i].getIntersect(ray, &inter) && inter.t < min_int.t) {
                min_int = inter;
            }
        }
        if (min_int.t != FLT_MAX) {
            *intersection = min_int;
            return true;
        }
        return false;
    }

    bool doesIntersect(const Ray& ray) const override {
        for (int i = 0; i < 12; i++) {
            if (sides[i].doesIntersect(ray)) {
                return true;
            }
        }
        return false;
    }

    float getSurfaceArea() const override {
        float sum = 0.0f;
        for (const Triangle& t : sides) sum += t.getSurfaceArea();
        return sum;
    }

    STPoint3 uniformSampleSurface(STVector3* normal) const override {
        // triangles with same areas are: (0,1,2,3), (4,5,6,7), (8,9,10,11)
       
        // chose a group out of those 3 based on ratio of the areas of its triangles.
        float A_cutoff = sides[0].getSurfaceArea();
        float B_cutoff = A_cutoff + sides[4].getSurfaceArea();
        float total = B_cutoff + sides[8].getSurfaceArea();
        
        // generate random in [0,total] and see if it's in [0,Acutoff], [Acutoff,Bcutoff],
        // or [Bcutoff,total] to choose a triangle group.
        // generate an int in [0,3] to pick a triangle within that group to sample from.
        float r = randFloat() * total;
        int k = rand() % 4;
        if (r < A_cutoff) {
            return sides[k].uniformSampleSurface(normal);
        } else if (r < B_cutoff) {
            return sides[4 + k].uniformSampleSurface(normal);
        } else {
            return sides[8 + k].uniformSampleSurface(normal);
        }
    }

private:
    void fillCob() {
        for (int ind = 0; ind < 3; ind++) {
            cob(ind,0) = this->i[ind];
            cob(ind,1) = this->j[ind];
            cob(ind,2) = this->k[ind];
        }
        cob(3,3) = 1.;
#if USE_EIGEN
        cob = cob.inverse().eval();
#else
        cob = cob.inverse();
#endif
    }
    void initTriangles(){
        sides[0] = Triangle(o, o + j, o + i + j);
        sides[1] = Triangle(o, o + i + j, o + i);
        sides[2] = Triangle(o + k, o + i + j + k, o + j + k);
        sides[3] = Triangle(o + k, o + i + k, o + i + j + k);
        sides[4] = Triangle(o, o + k, o + j + k);
        sides[5] = Triangle(o, o + j + k, o + j);
        sides[6] = Triangle(o + i, o + i + j + k, o + i + k);
        sides[7] = Triangle(o + i, o + i + j, o + i + j + k);
        sides[8] = Triangle(o, o + i, o + i + k);
        sides[9] = Triangle(o, o + i + k, o + k);
        sides[10] = Triangle(o + j, o + i + j + k, o + i + j);
        sides[11] = Triangle(o + j, o + j + k, o + i + j + k);
    }

    STPoint3 o;
    STVector3 i, j, k;
    STTransform4 cob;
    Triangle sides[12];
};

#endif
