#ifndef SCENE_H
#define SCENE_H

#include "st.h"
#include "RayTrace.h"

#include <iostream>

class LightDistribution;

class Scene
{
public:
    Scene();
    virtual ~Scene();

    void Render();
    STColor3f TraceRay(const Ray& ray, int bounce = -1);
    Intersection* Intersect(const Ray& ray, /*result*/SceneObject *& object);

    std::string info();

    void initializeSceneFromScript(std::string sceneFilename);
    void buildAccelStructures(std::string& accel);

    ////ray tracing APIs
    void rtClear();
    void rtCamera(const STPoint3& eye, const STVector3& up, const STPoint3& lookAt, float fovy, float aspect);
    void rtOutput(int imgWidth, int imgHeight, const std::string& outputFilename);
    void rtBounceDepth(int depth);
    void rtShadowBias(float bias);
    void rtSampleRate(int sample_rate);
    void rtPushMatrix();
    void rtPopMatrix();
    void rtLoadMatrix(const STTransform4& mat);
    void rtRotate(float rx, float ry, float rz);
    void rtScale(float sx, float sy, float sz);
    void rtTranslate(float tx, float ty, float tz);
    void rtSphere(const STPoint3& center, float radius);
    void rtTriangle(const STPoint3& v1, const STPoint3& v2, const STPoint3& v3);
    void rtTriangle(const STPoint3& v1, const STPoint3& v2, const STPoint3& v3, const STPoint2& uv1, const STPoint2& uv2, const STPoint2& uv3);
    void rtBox(const STPoint3& o, const STPoint3& x, const STPoint3& y, const STPoint3& z);
    void rtBox(const STPoint3& center, const STVector3& size);
    void rtCylinder(const STPoint3& A, const STPoint3 B, float radius);
    void rtParticipatingMedia(const STPoint3& center, const STVector3& size, const std::string& file_name);
    void rtCompound(char c);
    void rtGroupObjects(int num);
    void rtTriangleMesh(const std::string& file_name, const bool& counter_clockwise, const bool& smoothed_normal);    ////read geometry from the obj file, and load the geometry, the current material and current texture to ray tracer
    void rtTriangleMeshWithMaterialAndTexture(const std::string& file_name, const bool& counter_clockwise, const bool& smoothed_normal);    ////read geometry, material, and texture from the obj file and load them to ray tracer
    void rtAmbientLight(const STColor3f& col);
    void rtPointLight(const STPoint3& loc, const STColor3f& col);
    void rtDirectionalLight(const STVector3& dir, const STColor3f& col);
    void rtAreaLight(const STPoint3& v1, const STPoint3& v2, const STPoint3& v3, const STColor3f& col);
    void rtMaterial(const Material& mat);
    void rtMaterial(const STColor3f& amb, const STColor3f& diff, const STColor3f& spec, const STColor3f& mirr, float shine);
    void rtTransparentMaterial(const STColor3f& amb, const STColor3f& diff, const STColor3f& spec, const STColor3f& mirr, float shine, const STColor3f& refr, float sn);
    void rtSetFocus(const STPoint3& focal);
    void rtSetApeture(float a);
    void rtUseShadow(bool use = true){ use_shadow = use; }
    void rtUseTransparentShadow(bool use = true){ use_shadow = use; use_transparent_shadow = use; }
    void rtAttenuation(float a){ attenuation_coefficient = a; }
    void rtLoadTexture(const std::string image_name, int& tex_index);
    void rtLoadTexture(STImage* texture_image, int& tex_index);
    void rtVolumetricTexture(VolumetricTexture* vt);
    void rtBindTexture(const int tex_id){ currTexIndex = tex_id; }
    void rtUnbindTexture(){ currTexIndex = -1; }

protected:
    std::vector<STTransform4> matStack;
    Material* currMaterial;
    int currTexIndex;
    bool use_shadow;
    bool use_transparent_shadow;
    std::vector<STImage*> textures;
    std::vector<VolumetricTexture*> volumetric_textures;

    int bounceDepth;
    float shadowBias;
    int width, height;
    std::string imageFilename;
    float focus;
    float aperture;
    float attenuation_coefficient;
    int sampleRate;

    Camera *camera;
    std::vector<SceneObject *> objects;
    std::vector<Light *> lights;
    std::vector<AreaLight *> areaLights;

    LightDistribution lightDistribution;

    ////texture
    STColor3f textureColor(const int texture_index, const STPoint2& uv);

    ////acceleration structures
    enum AccelStructure{ NONE, AABB_TREE, UNIFORM_GRID } accel_structure;
    std::vector<AABBTree*> aabb_trees;
    UniformGrid* uniform_grid;

    ////intersection functions with acceleration structures
    Intersection* IntersectionNoAccelStructure(const Ray& ray, /*result*/SceneObject*& object);
    Intersection* IntersectAABBTree(const Ray& ray, /*result*/SceneObject*& object);
    Intersection* IntersectUniformGrid(const Ray& ray, /*result*/SceneObject*& object);

    void buildAABBTrees();	////build the entire scene as an aabb-tree
    void buildUniformGrids();	////build the entire scene as a uniform grid
    void fillLights(std::vector<Light *>& lights, Intersection* inter);
    void fillLightsWithAttenuation(std::vector<Light *>& lights, std::vector<STColor3f>& attenuations, Intersection* inter);
    STColor3f traceShadowRay(const Ray& ray, const Light& light);
    void getObjectsAABB(const std::vector<SceneObject*>& objs, /*result*/AABB& aabb);

    void generateEyeSubpath(float u, float v, std::vector<Vertex>& vertices, STColor3f* C_0t_sum);

    void generateLightSubpath(std::vector<Vertex>& vertices);
};



class LightDistribution {
public:
    LightDistribution() {}

    void init(std::vector<SceneObject*> objects) {
        // build list of light objects and their powers (max component) to be used
        // when picking the light object to sample y0 from.
        lightObjects.clear();
        lightPowers.clear();
        powerTotal = 0.f;
        for (SceneObject* o : objects) {
            if (o->isLight) {
                lightObjects.push_back(o);
                float power = o->emittedPower.maxComponent();
                lightPowers.push_back(power);
                powerTotal += power;
            }
        }
    }

    void sample_y0(STPoint3* point, STVector3* normal, Bsdf const** bsdf, float* Pa, STColor3f* Le0) {
        // choose a light source to sample based on max component of emitted power
        float r = (float)rand() / RAND_MAX * powerTotal;
        int chosen_i = 0;
        while (chosen_i < lightPowers.size() - 1) {
            r -= lightPowers[chosen_i];
            if (r < 0.f) break;
            chosen_i++;
        }
        float Pa_y0_multiplier = lightPowers[chosen_i] / powerTotal;

        // choose y0 by sampling selected light source, scale the resulting Pa
        float Pa_y0_obj;
        lightObjects[chosen_i]->sample_y0(point, normal, bsdf, &Pa_y0_obj, Le0);
        *Pa = Pa_y0_multiplier * Pa_y0_obj;
    }

    float Pa_y0(const SceneObject* light) {
        float Pa_y0_multiplier = light->emittedPower.maxComponent() / powerTotal;
        return Pa_y0_multiplier * light->Pa();
    }

    float Psig_y0_y1() {
        return 1.f / M_PI;
    }

private:
    // replaces lights and areaLights; is built from lights in render()
    std::vector <SceneObject*> lightObjects;

    // cached for randomly selecting a light based on power (max component)
    std::vector<float> lightPowers;
    float powerTotal;

};

#endif //SCENE_H

