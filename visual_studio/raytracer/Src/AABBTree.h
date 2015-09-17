//#####################################################################
// Stanford CS148 Ray Tracer Example Code
// Copyright 2013, Yue Chen
//#####################################################################

#ifndef AABBTree_h
#define AABBTree_h

#include "st.h"
#include "Shape.h"
#include "SceneObject.h"
#include "AABB.h"
#include <iostream>
#include <vector>
#include <algorithm>

class AABBTreeNode
{
public:
	AABBTreeNode(SceneObject* obj, const AABB& aabb);
	AABBTreeNode(std::vector<SceneObject*>& objs, int method=2);
	~AABBTreeNode();

    SceneObject* getIntersectionWithObject(const Ray& ray, Intersection* inter);
    bool doesIntersect(const Ray& ray);

    ////two ways to initialize the AABB tree
	static const int VOL = 1;
	static const int NUM = 2;

	AABB aabb;
	SceneObject* object;
	AABBTreeNode* left;
	AABBTreeNode* right;
};

class AABBTree //: public SceneObject
{
    //using SceneObject::name;
public:
    AABBTreeNode* root;

    AABBTree(const std::vector<SceneObject*>& objs) {
        //name="aabb_tree";

        // make copy of objs since AABBTreeNode constructor modifies them
        std::vector<SceneObject*> objsCopy(objs.size());
        for (int i = 0; i < objs.size(); i++) objsCopy[i] = objs[i];

        root = new AABBTreeNode(objsCopy);
		//std::cout<<"aabb tree, node number: "<<objs.size()<<std::endl;
    }
    ~AABBTree() {
        if(root!=NULL)delete root;
    }

    bool doesIntersect(const Ray& ray)  {
        return root->doesIntersect(ray);
    }

    SceneObject* getIntersectionWithObject(const Ray& ray, Intersection* inter) {
        return root->getIntersectionWithObject(ray, inter);
    }
};

#endif 
