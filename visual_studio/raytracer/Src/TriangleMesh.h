#ifndef RayTracer_TriangleMesh_h
#define RayTracer_TriangleMesh_h

#include <memory>

#include "Shape.h"
#include "Triangle.h"
#include "Sphere.h"
#include "AABB.h"
#include "AABBTree.h"
#include "STTriangleMesh.h"


class TriangleMesh : public Shape {
public:
	TriangleMesh(STTriangleMesh& mesh_input,bool _counter_clockwise=true,bool calculate_smoothed_normal=false,
		bool read_normal_from_obj=true,bool read_tex_coord_from_obj=true,bool _use_accel_structure=true)
		:counter_clockwise(_counter_clockwise),mesh(&mesh_input), use_accel_structure(_use_accel_structure),
        boundingSphere(STPoint3(0.f, 0.f, 0.f), 0.f)
	{
		this->name = "triangle_mesh";
		//maxInt = 1;
		if(calculate_smoothed_normal){
			if(!counter_clockwise){
				for(int i=0;i<(int)mesh->mFaces.size();i++){
					for(int d=0;d<3;d++){
						*(mesh->mFaces[i]->normals[d])*=-1.;
					}
				}
			}
		}

        std::vector<STPoint3> points;

		for(int i=0;i<(int)(mesh->mFaces.size());i++){
			////calculate position
			STPoint3 v[3];
			for(int d=0;d<3;d++){v[d]=mesh->mFaces[i]->v[d]->pt;}
			if(!counter_clockwise){STPoint3 tmp=v[1];v[1]=v[2];v[2]=tmp;}
			
            ////calculate normal
            STVector3 n[3];
            for(int d=0;d<3;d++){
                if(mesh->mFaces[i]->normals[d]!=0)n[d]=*(mesh->mFaces[i]->normals[d]);
                else n[d]=mesh->mFaces[i]->normal;
            }
            //if(read_normal_from_obj||calculate_smoothed_normal){
            //for(int d=0;d<3;d++){n[d]=*(mesh.mFaces[i]->normals[d]);}
            //}
            //else{
            //for(int d=0;d<3;d++){n[d]=mesh.mFaces[i]->normal;}
            //}
            if(!counter_clockwise){STVector3 tmp=n[1];n[1]=n[2];n[2]=tmp;}

            ////calculate tex coordinate
            STPoint2 vt[3];
            for(int d=0;d<3;d++){
                if(mesh->mFaces[i]->texPos[d]!=0)vt[d]=*(mesh->mFaces[i]->texPos[d]);
            }
            //if(read_tex_coord_from_obj){
            //for(int d=0;d<3;d++){vt[d]=*(mesh.mFaces[i]->texPos[d]);}
            //}

			//triangles.emplace_back(v[0],v[1],v[2],n[0],n[1],n[2],vt[0],vt[1],vt[2]);
            triangles.push_back(new TriangleMeshTriangle(Triangle(v[0], v[1], v[2], n[0], n[1], n[2], vt[0], vt[1], vt[2])));

            points.push_back(v[0]);
            points.push_back(v[1]);
            points.push_back(v[2]);
		}
        

        if (!use_accel_structure) {
            // calculate bounding sphere (Ritter's algorithm)

            // randomly choose point X
            const STPoint3& x = points[rand() % points.size()];
            // find furthest point Y from X
            float max_dist = 0.f;
            const STPoint3* y = NULL;
            for (const STPoint3& p : points) {
                float dist = (p - x).Length();
                if (dist > max_dist) {
                    max_dist = dist;
                    y = &p;
                }
            }
            // find furthest point Z from Y 
            max_dist = 0.f;
            const STPoint3* z = NULL;
            for (const STPoint3& p : points) {
                float dist = (p - *y).Length();
                if (dist > max_dist) {
                    max_dist = dist;
                    z = &p;
                }
            }

            // create initial bounding sphere
            STPoint3 c = (*y + *z) * 0.5f;
            float r = (*y - *z).Length() * 0.5f;

            // enlarge bounding sphere for every point found outside of it
            for (const STPoint3& p : points) {
                float c_p_dist = (p - c).Length();
                if (c_p_dist > r) {
                    // change sphere so that it covers previous sphere and point p
                    float r_change = 0.5f * (c_p_dist - r);
                    r += r_change;
                    c += r_change * ((p - c) / c_p_dist);
                }
            }

            boundingSphere.setCenter(c);
            boundingSphere.setRadius(r);

        } else {

			std::vector<SceneObject*> triangles_copy(triangles.size());
			for(int i=0;i<(int)triangles.size();i++){triangles_copy[i]=triangles[i];}
            aabb_tree = new AABBTree(triangles_copy);
		}
	}

    ~TriangleMesh()
    {
        for(int i=0;i<(int)triangles.size();i++){
            delete triangles[i];
        }
        if(aabb_tree) delete aabb_tree;
        //delete &mesh;
    }


	/*Intersection* getIntersect(const Ray& ray);
	bool doesIntersect(const Ray& ray);
	AABB* getAABB();

	Intersection** getIntersections(const Ray& ray){return NULL;}
	bool isInsideOpen(const STPoint3& pt) { return false; }
	bool isInsideClosed(const STPoint3& pt) { return false; }*/


	
    bool getIntersect(const Ray& ray, Intersection* intersection) const override;
    bool doesIntersect(const Ray& ray) const override;

	bool counter_clockwise;
private:
	std::unique_ptr<STTriangleMesh> mesh;
	//std::vector<Triangle> triangles;
    std::vector<SceneObject*> triangles;
    AABBTree* aabb_tree;
	bool use_accel_structure;

    Sphere boundingSphere;
};

#endif
