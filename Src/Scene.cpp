#define NOMINMAX
#include <Windows.h>

#include "Scene.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <memory>
#include <limits>



Scene::Scene()
    :currMaterial(NULL), currTexIndex(-1), use_shadow(true), use_transparent_shadow(false), attenuation_coefficient(1.f), camera(NULL), accel_structure(NONE), uniform_grid(NULL)
{
    rtSampleRate(1);
}

Scene::~Scene()
{
    if (currMaterial != NULL)delete currMaterial;
    if ((int)textures.size() > 0)for (int i = 0; i < (int)textures.size(); i++)delete textures[i];
    if ((int)volumetric_textures.size() > 0)for (int i = 0; i < (int)volumetric_textures.size(); i++)delete volumetric_textures[i];
    if (camera != NULL)delete camera;
    if ((int)objects.size() > 0)for (int i = 0; i < (int)objects.size(); i++)delete objects[i];
    if ((int)lights.size() > 0)for (int i = 0; i < (int)lights.size(); i++)delete lights[i];
    if ((int)areaLights.size() > 0)for (int i = 0; i < (int)areaLights.size(); i++)delete areaLights[i];
    if ((int)aabb_trees.size() > 0)for (int i = 0; i < (int)aabb_trees.size(); i++)delete aabb_trees[i];
    if (uniform_grid != NULL)delete uniform_grid;
}

std::string Scene::info() {
    std::stringstream out;
    out << "\nobject number: " << objects.size() << std::endl;
    for (int i = 0; i < (int)objects.size(); i++) {
        out << "obj " << i << ": " << objects[i]->name << std::endl;
    }

    out << "\nlight number: " << lights.size() << std::endl;
    for (int i = 0; i < (int)lights.size(); i++) {
        out << "light " << i << ": " << lights[i]->name << std::endl;
    }
    out << "\narea light number: " << areaLights.size() << std::endl;
    for (int i = 0; i < (int)areaLights.size(); i++) {
        out << "area light " << i << ": " << areaLights[i]->name << std::endl;
    }
    return out.str();
}



#define MIN_SUBPATH_LENGTH 3

// intersections[i] = z_i
// p_sig[0] = Pa(z0),   p_sig[i] = Psig(z_(i-1) -> z_i)
// aE[i] = aE_i
// q[i] = q corresponding to Psig(z_(i-1) -> z_i), or continuation probably at vertex z_(i-1)
// returns nE: num vertices

int Scene::generateEyeSubpath(float u, float v, std::vector<Vertex>& vertices, STColor3f* C_0t_sum) {

    







    // contributions Cst where s=0; this accumulates whenver this subpath runs into a light.
    *C_0t_sum = STColor3f(0.f);

    STColor3f f_z1i_zi_zi1;
    float p_sig_zi_zi1;
    float q_zi_zi1;
    Ray zi_zi1;
    std::unique_ptr<Intersection> zi_inter;
    SceneObject* zi_object;
    STColor3f aE_i1;

    // cos of angle between w and normal, where w is the last direction
    // chosen by sample_f
    float cos_sampled_w;
    
    aE_i1 = STColor3f(1.f);                        // aE_1 = 1;  this should work instead of C?

    // vertex z0 = eye
    vertices.emplace_back(Vertex(Intersection(0.f, camera->getEye(), camera->getLook()), NULL));
    vertices.back().alpha = aE_i1;

    
    /* // vertex z0 = xe
    intersections.push_back(InterSectionBsdf(Intersection(0.f, camera->getEye(), camera->getLook()), NULL));
    p_sig.push_back(1.0f);          // Pa(z0) = delta(z0 - xe)
    G.push_back(1.0f);              // G not really defined, but set it so that p_sig[0] * G[0] = Pa(z0)
    q.push_back(1.0f);
    aE.push_back(STColor3f(1.f));   // aE_1 = 1;  this should work instead of C?
    */

    // ray z0_z1 goes through (u,v) on img plane
    camera->generateRay(zi_zi1, u, v);
    zi_zi1.d.Normalize();               // generateRay does not normalize Ray::d for some reason
    f_z1i_zi_zi1 = STColor3f(1.f);      // We1(z0->z1) = 1; this should work instead of 1/C?
    p_sig_zi_zi1 = camera->Psig_cosW(u, v, &cos_sampled_w);  // Psig(z0->z1) depends on camera properties
    q_zi_zi1 = 1.f;


    vertices.back().qPsig_adj = q_zi_zi1 * p_sig_zi_zi1;


    int i = 1;      // current vertex
    while (true) {
        // intersect zi_zi1 with scene to find next vertex zi1
        zi_inter.reset(Intersect(zi_zi1, zi_object));
        if (!zi_inter) {
            // zi_zi1 didn't hit anything; terminate path at current zi
            return i + 1;
        }

        i++;
        
        // calculate G(zi<->zi1)
        float r = zi_inter->t;
        float cos_intersected_w = fabsf(STVector3::Dot(-zi_zi1.d, zi_inter->normal));
        float G_z1i_zi = cos_sampled_w * cos_intersected_w / (r*r);
        

        // calculate new alpha
        aE_i1 *= (f_z1i_zi_zi1 / (q_zi_zi1 * p_sig_zi_zi1));


        // record new vertex
        vertices.emplace_back(Vertex(*zi_inter, zi_object->bsdf));
        vertices.back().w_prev = -zi_zi1.d;
        vertices.back().alpha = aE_i1;
        vertices.back().qPsig_adj = 

        /*
        // record new vertex zi
        intersections.push_back(InterSectionBsdf(*zi, zi_object->bsdf));
        p_sig.push_back(p_sig_zi_zi1);
        G.push_back(G_zi_zi1);
        q.push_back(q_zi_zi1);
        aE.push_back(aE.back() * f_z1i_zi_zi1 / (q_zi_zi1 * p_sig_zi_zi1));
        */
        

        // if intersection is a light, then calculate contribution C_0t
        if (zi_object->isLight) {
            // C*_0t = aL_0 * c_0t * aE_t = 1 * Le(zi->z(i-1)) * aE_t
            STColor3f Cs_0t = zi_object->Le() * aE.back();

            // calculate w_0t = 1 / ((p1/p0)^2 + (p2/p0)^2 + .. + (pt/p0))
            
            float pi_over_p0 =
        }

        // choose next direction for ray zi_zi1 by sampling BSDF at zi.
        // wo_w = -zi_zi1.d,  reverse direction of the most recent zi_zi1 ray.
        // wi_w will become the direction of the next zi_zi1 ray
        STVector3 wi_w;
        f_z1i_zi_zi1 = intersections.back().sample_f(-zi_zi1.d, &wi_w, &p_sig_zi_zi1, &cos_sampled_w);
        
        // after the path reaches MIN_SUBPATH_LENGTH + 1 vertices, terminate the path
        // with some probability.
        if (i >= MIN_SUBPATH_LENGTH) {
            // probabilistically determine if eye subpath should terminate at zi
            // use the max component of f_z1i_zi_zi1 in f/p_sig evaluation
            q_zi_zi1 = (std::min)(f_z1i_zi_zi1.maxComponent() / p_sig_zi_zi1, 1.f);
            if ((float)rand() / RAND_MAX >= q_zi_zi1) {
                return i + 1;
            }
        }

        // update zi_zi1 to start from current zi and shoot in direction wi_w
        zi_zi1.at = zi_inter->point;
        zi_zi1.d = wi_w;
    }

    // go backwards through path and calculate Pa_next for all vertices

}


void Scene::generateLightSubpath(std::vector<Vertex>& vertices) {

    // choose a light source to sample based on max component of emitted power
    float r = (float)rand() / RAND_MAX * powerTotal;
    int chosen_i = 0;
    while (chosen_i < lightPowers.size() - 1) {
        r -= lightPowers[chosen_i];
        if (r < 0.f) break;
        chosen_i++;
    }
    float Pa_y0_multiplier = lightPowers[chosen_i] / powerTotal;

    // choose y0 by sampling selected light source
    STPoint3 y0_point;
    STVector3 y0_n;
    const Bsdf* y0_bsdf;
    float Pa_y0_obj;
    STColor3f Le0_y0;
    lightObjects[chosen_i]->sample_y0(&y0_point, &y0_n, &y0_bsdf, &Pa_y0_obj, &Le0_y0);
    float Pa_y0 = Pa_y0_multiplier * Pa_y0_obj;
    STColor3f alpha_1 = Le0_y0 / Pa_y0;

    // record y0
    vertices.emplace_back(Vertex(Intersection(0.f, y0_point, y0_n)));
    vertices.back().bsdf = y0_bsdf;
    //vertices.back().w_prev      // not defined for y0
    vertices.back().alpha = alpha_1;
    //vertices.back().G_prev      // not defined for y0

    STVector3 w;
    STPoint3 point = y0_point;
    STColor3f alpha = alpha_1;

    while (true) {

        // choose next direction w
        STVector3 w_old = w;
        float Psig;
        float cos_sampled_w;
        STColor3f f = vertices.back().sample_f(-w_old, &w, &Psig, &cos_sampled_w);

        // after the path reaches MIN_SUBPATH_LENGTH + 1 vertices, terminate the path
        // with some probability (1-q) where q = f / Psig for the next direction chosen.
        float q = 1.f;
        if (vertices.size() >= MIN_SUBPATH_LENGTH + 1) {
            q = std::min(f.maxComponent() / Psig, 1.f);
            if ((float)rand() / RAND_MAX >= q) {
                return;
            }
        }

        // find intersection between chosen direction and scene.
        Ray w_ray(point, w);
        SceneObject* inter_obj;
        Intersection* inter = Intersect(w_ray, inter_obj);
        if (!inter) {
            // ray didn't hit anything; terminate path
            return;
        }

        // calculate qPsig_adj for previous vertex, now that we know the direction
        // from it to the new vertex
        vertices.back().qPsig_adj = q * Psig;

        // calculate new alpha
        alpha *= (f / (q * Psig));

        // calculate G between new vertex and prev vertex
        float r = inter->t;
        float cos_intersected_w = fabsf(STVector3::Dot(-w, inter->normal));
        float G_prev = cos_sampled_w * cos_intersected_w / (r*r);

        // record new vertex
        vertices.emplace_back(Vertex(*inter));
        vertices.back().bsdf = inter_obj->bsdf;
        vertices.back().w_prev = -w;
        vertices.back().alpha = alpha;
        vertices.back().G_prev = G_prev;

        delete inter;
        inter = NULL;
    }
}

/*
int Scene::generateLightSubpath(std::vector<InterSectionBsdf>& intersections,
    std::vector<float>& p_sig, std::vector<float>& G, std::vector<STColor3f>& aL,
    std::vector<float>& q) {

    aL.push_back(STColor3f(1.f));         // aL_0 = 1

    // choose a light source to sample based on max component of emitted power
    float r = (float)rand() / RAND_MAX * powerTotal;
    int chosen_i = 0;
    while (true) {
        r -= lightPowers[chosen_i];
        if (r <= 0.f) break;
        chosen_i++;
    }
    float p_a_y0_multiplier = lightPowers[chosen_i] / powerTotal;


    // get y0 by sampling chosen light source
    InterSectionBsdf y0_intersection;
    float p_a_y0_object;
    STColor3f Le0_y0;
    lightObjects[chosen_i]->sample_y0(&y0_intersection, &p_a_y0_object, &Le0_y0);

    float p_a_y0 = p_a_y0_multiplier * p_a_y0_object;

    intersections.push_back(y0_intersection);
    p_sig.push_back(p_a_y0);        // Pa(y0)
    G.push_back(1.0f);              // G not really defined, but set it so that p_sig[0] * G[0] = Pa(y0)
    q.push_back(1.0f);
    aL.push_back(STColor3f(Le0_y0 / p_a_y0));   // aL_1 = Le0_y0 / Pa(y0)


    Ray yi_yi1;
    SceneObject* yi_object;
    std::unique_ptr<Intersection> yi(new Intersection(y0_intersection.getIntersection()));
    STColor3f f_y1i_yi_yi1;
    float p_sig_yi_yi1;
    float q_yi_yi1;

    // cos of angle between w and normal, where w is the last direction
    // chosen by sample_f
    float cos_sampled_w;

    int i = 0;      // current vertex
    while (true) {

        // choose next direction for ray yi_yi1 by sampling BSDF at yi.
        // wo_w = -yi_yi1.d,  reverse direction of the most recent yi_yi1 ray.
        // wi_w will become the direction of the next yi_yi1 ray
        STVector3 wi_w;
        f_y1i_yi_yi1 = intersections.back().sample_f(-yi_yi1.d, &wi_w, &p_sig_yi_yi1, &cos_sampled_w);

        // after the path reaches MIN_SUBPATH_LENGTH + 1 vertices, terminate the path
        // with some probability.
        if (i >= MIN_SUBPATH_LENGTH) {
            // probabilistically determine if eye subpath should terminate at yi
            // use the max component of f_y1i_yi_yi1 in f/p_sig evaluation
            q_yi_yi1 = (std::min)(f_y1i_yi_yi1.maxComponent() / p_sig_yi_yi1, 1.f);
            if ((float)rand() / RAND_MAX >= q_yi_yi1) {
                return i + 1;
            }
        }

        // update yi_yi1 to start from current yi and shoot in direction wi_w
        yi_yi1.at = yi->point;
        yi_yi1.d = wi_w;

        // --------------------------------------------------------------------------------------

        // intersect yi_yi1 with scene to find next vertex yi1
        yi.reset(Intersect(yi_yi1, yi_object));
        if (!yi) {
            // yi_yi1 didn't hit anything; terminate path at current yi
            return i + 1;
        }

        // calculate G(zi<->zi1)
        float r = yi->t;
        float cos_intersected_w = fabsf(STVector3::Dot(-yi_yi1.d, yi->normal));
        float G_zi_zi1 = cos_sampled_w * cos_intersected_w / (r*r);

        // record new intersection, Psig, and aE
        intersections.push_back(InterSectionBsdf(*yi, yi_object->bsdf));
        p_sig.push_back(p_sig_yi_yi1);
        G.push_back(G_zi_zi1);
        q.push_back(q_yi_yi1);
        aL.push_back(aL.back() * f_y1i_yi_yi1 / (q_yi_yi1 * p_sig_yi_yi1));

        i++;
    }
}*/





void Scene::Render() {
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

    std::cout << "------------------ray tracing started------------------" << std::endl;
    STImage *im = new STImage(width, height);
    STImage *im_L = new STImage(width, height);     // light image (for s=1 paths)
    ImagePlane imPlane = ImagePlane(width, height);

    int percent = 0, computed = 0;
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {

            // work on pixel (x, y)

            // sampleRate^2 stratified estimates (sample-path groups) per pixel
            for (int a = 0; a < sampleRate; a++) {
                for (int b = 0; b < sampleRate; b++) {

                    // offset (x,y) by between [a/sampleRate, b/sampleRate] and [(a+1)/sampleRate, (b+1)/sampleRate)]
                    // those are the bounds for the (a,b) subpixel within pixel (x,y)
                    float xs = (float)x + (((float)a + (float)rand() / RAND_MAX) / sampleRate);
                    float ys = (float)y + (((float)b + (float)rand() / RAND_MAX) / sampleRate);
                    
                    // convert to screen coords (u,v)
                    float u = xs / width;
                    float v = ys / height;
                    

                    // generate eye subpath thru (u,v)
                    int nE;
                    std::vector<InterSectionBsdf> intersections_E;
                    std::vector<float> p_sig_E;
                    std::vector<STColor3f> aE;
                    std::vector<float> q_E;
                    nE = generateEyeSubpath(u, v, intersections_E, p_sig_E, aE, q_E);

                    // generate light subpath
                    int nL;
                    std::vector<InterSectionBsdf> intersections_L;
                    std::vector<float> p_sig_L;
                    std::vector<STColor3f> aL;
                    std::vector<float> q_L;
                    nL = generateLightSubpath(intersections_L, p_sig_L, aL, q_L);
                    

                    // calculate s=1 contributions: (light image paths)
                    
                }
            }

            /*
            int subimage_w0 = 242; int subimage_w1 = 242;
            int subimage_h0 = 310; int subimage_h1 = 310;
            bool use_subimage = false;

            int percent = 0, computed = 0;
            for (int w = 0; w < width; w++) {
                for (int h = 0; h < height; h++) {
                    if (use_subimage && (w<subimage_w0 || w>subimage_w1 || h<subimage_h0 || h>subimage_h1)){
                        im->SetPixel(w, h, STColor4ub(0, 0, 255, 255));
                        continue;
                    }
                    if (use_subimage)std::cout << "trace_ray: " << w << ", " << h << std::endl;

                    std::vector<Ray *>* rays = imPlane.getRays(camera, w, h, sampleRate, focus, aperture);
                    STColor3f avg = STColor3f(0.f, 0.f, 0.f);
                    for (int i = 0; i < (int)rays->size(); i++) {
                        STColor3f ray_color = TraceRay(*rays->at(i));
                        if (use_subimage)std::cout << "ray_color: " << ray_color.r << ", " << ray_color.g << ", " << ray_color.b << std::endl;
                        avg += ray_color;
                        delete rays->at(i);
                    }
                    im->SetPixel(w, h, STColor4ub(avg / (float)rays->size()));
                    delete rays;

                    computed++;
                    if (100 * computed / (width * height) > percent) {
                        percent++;
                        std::cout << percent << "% ";
                    }
                }
            }*/

            im->Save(imageFilename);
            delete im;
            std::cout << "------------------ray tracing finished------------------" << std::endl;
        }
    }
}

STColor3f Scene::TraceRay(const Ray &ray, int bounce) {
    if (bounce == bounceDepth){
        return STColor3f();
    }
    SceneObject *object;
    Intersection* inter = Intersect(ray, object);
    if (!inter){
        return STColor3f();
    }

    if (object->shape->name == "light") {
        delete inter;
        return object->material.diffuse;
    }

    STVector3 d = ray.d;
    d.Normalize();

    STVector3 reflected_d = STVector3::Dot(-1. * ray.d, inter->normal) * 2 * inter->normal + ray.d;
    Ray* refracted = NULL;
    Ray* reflected = new Ray(inter->point, reflected_d, shadowBias);

    float cos_theta_1 = STVector3::Dot(inter->normal, d);


    ////participating media
    if (object->material.isParticipatingMedia()) {
        if (cos_theta_1 > 0) { //exiting participating media
            refracted = new Ray(inter->point, ray.d, shadowBias);
            STColor3f result = TraceRay(*refracted, bounce + 1);
            delete refracted;
            delete reflected;
            return result;
        } else { //entering participating media
            refracted = new Ray(inter->point, ray.d, shadowBias);
            STColor3f color = TraceRay(*refracted, bounce + 1);
            //calculate attenuation based on distance ray traveled through medium
            float atten = object->material.participatingMediaAttenuation(refracted->e, refracted->at(inter->t), *(object->aabb));
            STColor3f atten_color(atten, atten, atten);
            STColor3f result = color * atten_color;
            delete refracted;
            delete reflected;
            return result;
        }
    }

    ////from inside to outside, n1>n2
    if (cos_theta_1 > 0 && !object->material.isOpaque()) {
        float dist = (ray.at(inter->t) - ray.e).Length();
        STColor3f atten = ((STColor3f(1, 1, 1) - object->material.refract) * attenuation_coefficient * (-dist)).Exp();
        Ray *refracted = object->material.refracted(ray, inter, shadowBias);
        if (refracted) {
            float nt = object->material.snell;
            float sin_theta_2 = nt * sqrt(1 - cos_theta_1 * cos_theta_1);
            float cos_theta_2 = sqrt(1 - sin_theta_2 * sin_theta_2);
            float r0 = pow((nt - 1) / (nt + 1), 2);
            float refl = r0 + (1 - r0) * pow(1 - cos_theta_2, 5);
            STColor3f reflected_ray_color = TraceRay(*reflected, bounce + 1);
            STColor3f refracted_ray_color = TraceRay(*refracted, bounce + 1);
            STColor3f result = atten * (reflected_ray_color * refl + refracted_ray_color * (1 - refl));
            delete reflected;
            delete refracted;
            delete inter;
            return result;
        } else {
            STColor3f reflected_ray_color = TraceRay(*reflected, bounce + 1);
            STColor3f result = atten * reflected_ray_color;
            delete reflected;
            delete inter;
            return result;
        }
    }

    std::vector<Light *> visibleLights;
    std::vector<STColor3f> attenuation;
    if (!use_transparent_shadow){ fillLights(visibleLights, inter); } else{ fillLightsWithAttenuation(visibleLights, attenuation, inter); }


    ////from outside to inside, n1<n2
    if (!object->material.isOpaque() && object->material.snell) {
        refracted = object->material.refracted(ray, inter, shadowBias);
        if (refracted) {
            float nt = object->material.snell;
            float r0 = pow((nt - 1) / (nt + 1), 2);
            float refl = r0 + (1 - r0) * pow(1 + cos_theta_1, 5);
            STColor3f reflected_ray_color = TraceRay(*reflected, bounce + 1);
            STColor3f refracted_ray_color = TraceRay(*refracted, bounce + 1);
            STColor3f result = reflected_ray_color * refl + refracted_ray_color * (1 - refl);
            delete inter;
            delete reflected;
            delete refracted;
            return result;
        } else {
            STColor3f result = TraceRay(*reflected, bounce + 1);
            delete inter;
            delete reflected;
            return result;
        }
    }

    STColor3f result;
    if (use_transparent_shadow){
        result = object->material.shade(inter, ray.d * -1., visibleLights, attenuation, object->material.isMatte() ? STColor3f() : TraceRay(*reflected, bounce + 1), refracted ? TraceRay(*refracted, bounce + 1) : STColor3f());
    } else{
        result = object->material.shade(inter, ray.d * -1., visibleLights, object->material.isMatte() ? STColor3f() : TraceRay(*reflected, bounce + 1), refracted ? TraceRay(*refracted, bounce + 1) : STColor3f());
    }
    if (object->texture_index != -1){
        STColor3f tex_color = textureColor(object->texture_index, inter->uv);
        result *= tex_color;
    }
    delete inter;
    delete reflected;
    if (refracted) delete refracted;
    return result;
}

////find the visible lights from the intersection
void Scene::fillLights(std::vector<Light *> &visibleLights, Intersection* inter) {
    if (!use_shadow){
        for (int i = 0; i < (int)lights.size(); i++){ visibleLights.push_back(lights[i]); }
        return;
    }

    SceneObject *dummy;
    for (int i = 0; i < (int)lights.size(); i++) {
        if (lights[i]->name == "ambient")visibleLights.push_back(lights[i]);
        else{
            STVector3 toL = lights[i]->direction(inter->point);
            Intersection* lightInt = Intersect(Ray(inter->point, toL, shadowBias), dummy);
            if (!lightInt ||   ////no obstacle between light and object
                ((lights[i]->name == "point" || lights[i]->name == "area") && (lightInt->t > 1. || lightInt->t < shadowBias)) ||
                (lights[i]->name == "directional" && lightInt->t < shadowBias)) { ////out of range between object and light
                visibleLights.push_back(lights[i]);
            }
            delete lightInt;
        }
    }
    for (int i = 0; i < (int)areaLights.size(); i++) {
        STVector3 toL = areaLights[i]->direction(inter->point);
        Intersection* lightInt = Intersect(Ray(inter->point, toL, shadowBias), dummy);

        if ((toL.x == 0. || toL.y == 0. || toL.z == 0.) || (!lightInt || (lightInt->t > 1. || lightInt->t < shadowBias))) {
            visibleLights.push_back(areaLights[i]);
        }
        delete lightInt;
    }
}

void Scene::fillLightsWithAttenuation(std::vector<Light *>& visibleLights, std::vector<STColor3f>& attenuations, Intersection* inter)
{
    if (!use_shadow){
        for (int i = 0; i < (int)lights.size(); i++){ visibleLights.push_back(lights[i]); }
        return;
    }

    for (int i = 0; i < (int)lights.size(); i++) {
        if (lights[i]->name == "ambient"){
            visibleLights.push_back(lights[i]);
            attenuations.push_back(STColor3f(1.f, 1.f, 1.f));
        } else{
            STVector3 toL = lights[i]->direction(inter->point);
            Ray* shadow_ray = new Ray(inter->point, toL, shadowBias);
            STColor3f atten = traceShadowRay(*shadow_ray, *lights[i]);

            if (atten.r != 0.f&&atten.g != 0.f&&atten.b != 0.f) {
                visibleLights.push_back(lights[i]);
                attenuations.push_back(atten);
            }
            delete shadow_ray;
        }
    }
    for (int i = 0; i < (int)areaLights.size(); i++) {
        STVector3 toL = areaLights[i]->direction(inter->point);
        Ray* shadow_ray = new Ray(inter->point, toL, shadowBias);
        STColor3f atten = traceShadowRay(*shadow_ray, *areaLights[i]);

        if (atten.r != 0.f&&atten.g != 0.f&&atten.b != 0.f) {
            visibleLights.push_back(areaLights[i]);
            attenuations.push_back(atten);
        }
        delete shadow_ray;
    }
}

STColor3f Scene::traceShadowRay(const Ray& ray, const Light& light)
{
    SceneObject *object = NULL;
    Intersection* inter = Intersect(ray, object);
    if (object != NULL){
        if (((light.name == "point" || light.name == "area") && (inter->t > 1. || inter->t < shadowBias))
            || (light.name == "directional" && inter->t < shadowBias)){
            delete inter;
            return STColor3f(1, 1, 1);
        }
        if (object->material.isOpaque()){
            delete inter;
            return STColor3f(0, 0, 0);
        }

        float cos_theta_1 = STVector3::Dot(inter->normal, ray.d);
        STColor3f atten(1.f, 1.f, 1.f);
        if (cos_theta_1 > 0) {
            float dist = (ray.at(inter->t) - ray.e).Length();
            atten *= ((STColor3f(1, 1, 1) - object->material.refract) * attenuation_coefficient * (-dist)).Exp();
        }

        Ray* continue_ray = new Ray(ray.at(inter->t), ray.e + ray.d - ray.at(inter->t), shadowBias);
        STColor3f continue_atten = traceShadowRay(*continue_ray, light);
        delete inter;
        delete continue_ray;
        return atten*continue_atten;
    } else{
        delete inter;
        return STColor3f(1, 1, 1);
    }
}

void Scene::rtClear()
{
    currMaterial = NULL;
    matStack.clear();
    matStack.push_back(STTransform4::Identity());
    focus = 0.;
}

void Scene::rtCamera(const STPoint3& eye, const STVector3& up, const STPoint3& lookAt, float fovy, float aspect)
{
    camera = new Camera(eye, up, lookAt, fovy, aspect);
}

void Scene::rtOutput(int imgWidth, int imgHeight, const std::string& outputFilename)
{
    width = imgWidth;
    height = imgHeight;
    imageFilename = outputFilename;
}

void Scene::rtBounceDepth(int depth)
{
    bounceDepth = depth;
}

void Scene::rtShadowBias(float bias)
{
    shadowBias = bias;
}

void Scene::rtSampleRate(int r)
{
    sampleRate = r;
    for (int i = 0; i < (int)areaLights.size(); i++) areaLights[i]->setSampleRate(r);
}

void Scene::rtPushMatrix()
{
    if (!matStack.empty()) matStack.push_back(matStack.back());
}

void Scene::rtPopMatrix()
{
    if (!matStack.empty()) matStack.pop_back();
}

void Scene::rtLoadMatrix(const STTransform4& mat)
{
    matStack.back() = mat;
}

void Scene::rtRotate(float rx, float ry, float rz)
{
    if (!matStack.empty()) {
        float conv = 3.14159265358979f / 180.f;
        STTransform4 M = matStack.back() * STTransform4::Rotation(rx * conv, ry * conv, rz * conv);
        matStack.pop_back();
        matStack.push_back(M);
    }
}

void Scene::rtScale(float sx, float sy, float sz)
{
    if (!matStack.empty()) {
        STTransform4 M = matStack.back() * STTransform4::Scaling(sx, sy, sz);
        matStack.pop_back();
        matStack.push_back(M);
    }
}

void Scene::rtTranslate(float tx, float ty, float tz)
{
    if (!matStack.empty()) {
        STTransform4 M = matStack.back() * STTransform4::Translation(tx, ty, tz);
        matStack.pop_back();
        matStack.push_back(M);
    }
}

void Scene::rtSphere(const STPoint3& center, float radius)
{
    objects.push_back(new SceneObject(new Sphere(center, radius), currMaterial, &matStack.back()));
}

void Scene::rtTriangle(const STPoint3& v1, const STPoint3& v2, const STPoint3& v3)
{
    objects.push_back(new SceneObject(new Triangle(v1, v2, v3), currMaterial, &matStack.back()));
}

void Scene::rtTriangle(const STPoint3& v1, const STPoint3& v2, const STPoint3& v3, const STPoint2& uv1, const STPoint2& uv2, const STPoint2& uv3)
{
    objects.push_back(new SceneObject(new Triangle(v1, v2, v3, uv1, uv2, uv3), currMaterial, &matStack.back(), currTexIndex));
}

void Scene::rtBox(const STPoint3& o, const STPoint3& x, const STPoint3& y, const STPoint3& z)
{
    objects.push_back(new SceneObject(new Box(o, x, y, z), currMaterial, &matStack.back()));
}

void Scene::rtBox(const STPoint3& center, const STVector3& size)
{
    objects.push_back(new SceneObject(new Box(center, size), currMaterial, &matStack.back()));
}

void Scene::rtCylinder(const STPoint3& A, const STPoint3 B, float radius)
{
    objects.push_back(new SceneObject(new Cylinder(A, B, radius), currMaterial, &matStack.back()));
}


void Scene::rtParticipatingMedia(const STPoint3& center, const STVector3& size, const std::string& file_name)
{
    Material* participating_media_material = new Material();
    objects.push_back(new SceneObject(new Box(center, size), currMaterial, &matStack.back()));
}

void Scene::rtCompound(char c)
{
    if (objects.size() > 1) {
        SceneObject *two = objects.back();
        objects.pop_back();
        SceneObject *one = objects.back();
        objects.pop_back();
        objects.push_back(new SceneObject(new CompoundShape(one->shape, two->shape, c, shadowBias), &one->material, &one->transform));
        delete one;
        delete two;
    }
}

void Scene::rtGroupObjects(int num) {
    SceneObject *prevObj = objects.back();
    objects.pop_back();
    ObjectGroup* boundVol = new ObjectGroup(prevObj);   ////add the bounding volume for the group
    for (int i = 0; i < num; i++) {
        ObjectGroup* bv = dynamic_cast<ObjectGroup*>(objects.back());
        if (bv != 0)boundVol->addSubObject(bv);    ////handle the case that objects.back() is already a ObjectGroup
        else boundVol->addSubObject(new ObjectGroup(objects.back()));
        objects.pop_back();
    }
    objects.push_back(boundVol);
}

void Scene::rtTriangleMesh(const std::string& file_name, const bool& counter_clockwise, const bool& smoothed_normal)
{
    std::vector<STTriangleMesh*> meshes;
    STTriangleMesh::LoadObj(meshes, file_name);
    for (int i = 0; i < (int)meshes.size(); i++) {
        objects.push_back(new SceneObject(new TriangleMesh(*meshes[i], counter_clockwise, smoothed_normal), currMaterial, &matStack.back(), currTexIndex));
    }
    //objects.push_back(new SceneObject(new TriangleMesh(file_name,counter_clockwise,smoothed_normal), currMaterial, &matStack.back(), currTexIndex));
}

void Scene::rtTriangleMeshWithMaterialAndTexture(const std::string& file_name, const bool& counter_clockwise, const bool& smoothed_normal)
{
    std::vector<STTriangleMesh*> meshes;
    STTriangleMesh::LoadObj(meshes, file_name);
    for (int i = 0; i < (int)meshes.size(); i++) {
        STTriangleMesh* st_mesh = meshes[i];
        int tex_index = -1; if (st_mesh->mSurfaceColorImg)rtLoadTexture(st_mesh->mSurfaceColorImg, tex_index);
        STColor3f amb(st_mesh->mMaterialAmbient[0], st_mesh->mMaterialAmbient[1], st_mesh->mMaterialAmbient[2]);
        STColor3f diff(st_mesh->mMaterialDiffuse[0], st_mesh->mMaterialDiffuse[1], st_mesh->mMaterialDiffuse[2]);
        STColor3f spec(st_mesh->mMaterialSpecular[0], st_mesh->mMaterialSpecular[1], st_mesh->mMaterialSpecular[2]);
        float shin = st_mesh->mShininess;
        Material* mat = new Material(amb, diff, spec, /*mirror*/STColor3f(), shin);
        objects.push_back(new SceneObject(new TriangleMesh(*meshes[i], counter_clockwise, smoothed_normal), mat, &matStack.back(), tex_index));
    }
}

void Scene::rtAmbientLight(const STColor3f& col)
{
    lights.push_back(new AmbientLight(col));
}

void Scene::rtPointLight(const STPoint3& loc, const STColor3f& col)
{
    lights.push_back(new PointLight(matStack.back() * loc, col));
}

void Scene::rtDirectionalLight(const STVector3& dir, const STColor3f& col)
{
    lights.push_back(new DirectionalLight(matStack.back() * dir, col));
}

void Scene::rtAreaLight(const STPoint3& v1, const STPoint3& v2, const STPoint3& v3, const STColor3f& col)
{
    areaLights.push_back(new AreaLight(matStack.back() * v1, matStack.back() * v2, matStack.back() * v3, col, sampleRate));
}

void Scene::rtMaterial(const STColor3f& amb, const STColor3f& diff, const STColor3f& spec, const STColor3f& mirr, float shine)
{
    if (currMaterial != NULL) delete currMaterial;
    currMaterial = new Material(amb, diff, spec, mirr, shine);
}

void Scene::rtMaterial(const Material& mat)
{
    if (currMaterial == NULL)currMaterial = new Material(mat.ambient, mat.diffuse, mat.specular, mat.mirror, mat.shininess, mat.refract, mat.snell, mat.volumetric_texture);
    else{
        currMaterial->ambient = mat.ambient;
        currMaterial->diffuse = mat.diffuse;
        currMaterial->specular = mat.specular;
        currMaterial->mirror = mat.mirror;
        currMaterial->shininess = mat.shininess;
        currMaterial->refract = mat.refract;
        currMaterial->snell = mat.snell;
        currMaterial->volumetric_texture = mat.volumetric_texture;
    }
}

void Scene::rtTransparentMaterial(const STColor3f& amb, const STColor3f& diff, const STColor3f& spec, const STColor3f& mirr, float shine, const STColor3f& refr, float sn)
{
    if (currMaterial != NULL) delete currMaterial;
    currMaterial = new Material(amb, diff, spec, mirr, shine, refr, sn);
}

void Scene::rtSetFocus(const STPoint3& focal)
{
    STPoint3 pt = matStack.back() * focal;
    focus = camera->getFocalRatio(pt);
}
void Scene::rtSetApeture(float a)
{
    aperture = a;
}

void Scene::buildAccelStructures(std::string& accel)
{
    if (accel == "aabb"){ accel_structure = AABB_TREE; } else if (accel == "grid"){ accel_structure = UNIFORM_GRID; } else{ accel_structure = NONE; }

    switch (accel_structure){
    case AABB_TREE:
        buildAABBTrees();
        break;
    case UNIFORM_GRID:
        buildUniformGrids();
        break;
    default:
        break;
    }
}

void Scene::buildAABBTrees()
{
    ////by default all objects are organized in ONE aabb tree
    accel_structure = AABB_TREE;
    AABBTree* aabb_tree = new AABBTree(objects);
    aabb_trees.push_back(aabb_tree);
}

void Scene::buildUniformGrids()
{
    accel_structure = UNIFORM_GRID;
    ////calculate the scene bounding box
    AABB scene_bounding_box; getObjectsAABB(objects, scene_bounding_box);

    ////default scene subdivision 
    int subdivision = 10; STVector3 edge_length = scene_bounding_box.edgeLength();
    float dx = AABB::getMax(edge_length.x, edge_length.y, edge_length.z) / (float)subdivision;
    int scene_subdivision[3] = { 1, 1, 1 };
    for (int d = 0; d < 3; d++)scene_subdivision[d] = (int)ceil(edge_length.Component(d) / dx);

    uniform_grid = new UniformGrid(objects, scene_bounding_box, scene_subdivision);
}

Intersection* Scene::Intersect(const Ray& ray, /*result*/SceneObject *& object)
{
    switch (accel_structure){
    case AABB_TREE:return IntersectAABBTree(ray, object);
    case UNIFORM_GRID:return IntersectUniformGrid(ray, object);
    default:return IntersectionNoAccelStructure(ray, object);
    }
}

Intersection* Scene::IntersectionNoAccelStructure(const Ray& ray, /*result*/SceneObject*& object)
{
    Intersection* min_inter = NULL;
    SceneObject* current_object = NULL;
    SceneObject* min_object = NULL;
    for (int i = 0; i < (int)objects.size(); i++) {
        SceneObject* obj = objects[i];
        Intersection *inter = obj->getIntersectionWithObject(ray, current_object);

        if (inter && (!min_inter || inter->t < min_inter->t) && ray.inRange(inter->t)) {
            if (min_inter) delete min_inter;
            min_inter = inter;
            min_object = current_object;
        } else delete inter;
    }
    object = min_object;
    return min_inter;
}

Intersection* Scene::IntersectAABBTree(const Ray& ray, /*result*/SceneObject*& object)
{
    Intersection* min_inter = NULL;
    SceneObject* current_object = NULL;
    SceneObject* min_object = NULL;
    for (int i = 0; i < (int)aabb_trees.size(); i++) {
        AABBTree* tree = aabb_trees[i];
        Intersection *inter = tree->getIntersectionWithObject(ray, current_object);

        if (inter && (!min_inter || inter->t < min_inter->t) && ray.inRange(inter->t)) {
            if (min_inter) delete min_inter;
            min_inter = inter;
            min_object = current_object;
        } else delete inter;
    }
    object = min_object;
    return min_inter;
}

Intersection* Scene::IntersectUniformGrid(const Ray& ray, /*result*/SceneObject*& object)
{
    return uniform_grid->getIntersectionWithObject(ray, object);
}

void Scene::getObjectsAABB(const std::vector<SceneObject*>& objs, /*result*/AABB& aabb)
{
    aabb.xmin = FLT_MAX; aabb.xmax = -FLT_MAX; aabb.ymin = FLT_MAX; aabb.ymax = -FLT_MAX; aabb.zmin = FLT_MAX; aabb.zmax = -FLT_MAX;
    for (int i = 0; i < (int)objects.size(); i++){
        AABB::combine(&aabb, objects[i]->aabb, &aabb);
    }
    float offset = aabb.maxEdgeLength()*.001f;
    aabb.enlarge(offset);
}

void Scene::initializeSceneFromScript(std::string sceneFilename)
{
    rtClear();

    std::ifstream sceneFile(sceneFilename.c_str());

    if (sceneFile.fail()){
        printf("Scene::initializeSceneFromScript - Could not find input scene file '%s'\n", sceneFilename.c_str());
        exit(1);
    }

    char line[1024];
    while (!sceneFile.eof()){
        sceneFile.getline(line, 1023);
        std::stringstream ss;
        ss.str(line);
        std::string command;
        ss >> command;

        if (command == "Camera") {
            float ex, ey, ez, ux, uy, uz, lx, ly, lz, f, a;
            ss >> ex >> ey >> ez >> ux >> uy >> uz >> lx >> ly >> lz >> f >> a;
            STPoint3 eye(ex, ey, ez);
            STVector3 up(ux, uy, uz);
            STPoint3 lookAt(lx, ly, lz);
            rtCamera(eye, up, lookAt, f, a);
        } else if (command == "Output") {
            int w, h;
            std::string fname;
            ss >> w >> h >> fname;
            rtOutput(w, h, fname);
        } else if (command == "BounceDepth") {
            int depth;
            ss >> depth;
            rtBounceDepth(depth);
        } else if (command == "ShadowBias") {
            float bias;
            ss >> bias;
            rtShadowBias(bias);
        } else if (command == "PushMatrix") {
            rtPushMatrix();
        } else if (command == "PopMatrix") {
            rtPopMatrix();
        } else if (command == "LoadMatrix") {
            STTransform4 mat;
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    ss >> mat[i][j];
                }
            }
            rtLoadMatrix(mat);
        } else if (command == "Rotate") {
            float rx, ry, rz;
            ss >> rx >> ry >> rz;
            rtRotate(rx, ry, rz);
        } else if (command == "Scale") {
            float sx, sy, sz;
            ss >> sx >> sy >> sz;
            rtScale(sx, sy, sz);
        } else if (command == "Translate") {
            float tx, ty, tz;
            ss >> tx >> ty >> tz;
            rtTranslate(tx, ty, tz);
        } else if (command == "Sphere") {
            float cx, cy, cz, r;
            ss >> cx >> cy >> cz >> r;
            STPoint3 center(cx, cy, cz);
            rtSphere(center, r);
        } else if (command == "Triangle") {
            float x1, y1, z1, x2, y2, z2, x3, y3, z3;
            ss >> x1 >> y1 >> z1 >> x2 >> y2 >> z2 >> x3 >> y3 >> z3;
            STPoint3 v[3];
            v[0] = STPoint3(x1, y1, z1);
            v[1] = STPoint3(x2, y2, z2);
            v[2] = STPoint3(x3, y3, z3);
            rtTriangle(v[0], v[1], v[2]);
        } else if (command == "Box") {
            float o1, o2, o3, x1, x2, x3, y1, y2, y3, z1, z2, z3;
            ss >> o1 >> o2 >> o3 >> x1 >> x2 >> x3 >> y1 >> y2 >> y3 >> z1 >> z2 >> z3;
            STPoint3 o = STPoint3(o1, o2, o3);
            STPoint3 x = STPoint3(x1, x2, x3);
            STPoint3 y = STPoint3(y1, y2, y3);
            STPoint3 z = STPoint3(z1, z2, z3);
            rtBox(o, x, y, z);
        } else if (command == "Cylinder") {
            float a1, a2, a3, b1, b2, b3, r;
            ss >> a1 >> a2 >> a3 >> b1 >> b2 >> b3 >> r;
            STPoint3 A(a1, a2, a3);
            STPoint3 B(b1, b2, b3);
            rtCylinder(A, B, r);
        } else if (command == "ParticipatingMedia") {
            float c1, c2, c3, s1, s2, s3;
            std::string file_name;
            ss >> c1 >> c2 >> c3 >> s1 >> s2 >> s3 >> file_name;
            STPoint3 center(c1, c2, c3);
            STVector3 size(s1, s2, s3);
            rtParticipatingMedia(center, size, file_name);
        } else if (command == "Compound") {
            char c;
            ss >> c;
            rtCompound(c);
        } else if (command == "Group") {
            int num;
            ss >> num;
            rtGroupObjects(num);
        } else if (command == "TriangleMesh") {
            std::string file_name; int counter_clockwise; int smoothed_normal;
            ss >> file_name >> counter_clockwise >> smoothed_normal;
            rtTriangleMesh(file_name, (counter_clockwise != 0), (smoothed_normal != 0));
        } else if (command == "AmbientLight") {
            float r, g, b;
            ss >> r >> g >> b;
            STColor3f col(r, g, b);
            rtAmbientLight(col);
        } else if (command == "PointLight") {
            float px, py, pz, r, g, b;
            ss >> px >> py >> pz >> r >> g >> b;
            STPoint3 pos(px, py, pz);
            STColor3f col(r, g, b);
            rtPointLight(pos, col);
        } else if (command == "DirectionalLight") {
            float dx, dy, dz, r, g, b;
            ss >> dx >> dy >> dz >> r >> g >> b;
            STVector3 dir(dx, dy, dz);
            dir.Normalize();
            STColor3f col(r, g, b);
            rtDirectionalLight(dir, col);
        } else if (command == "AreaLight") {
            float x1, y1, z1, x2, y2, z2, x3, y3, z3, r, g, b;
            ss >> x1 >> y1 >> z1 >> x2 >> y2 >> z2 >> x3 >> y3 >> z3 >> r >> g >> b;
            STPoint3 v[3];
            v[0] = STPoint3(x1, y1, z1);
            v[1] = STPoint3(x2, y2, z2);
            v[2] = STPoint3(x3, y3, z3);
            STColor3f col(r, g, b);
            rtAreaLight(v[0], v[1], v[2], col);
        } else if (command == "Material") {
            float ra, ga, ba, rd, gd, bd, rs, gs, bs, rr, gr, br, shine;
            ss >> ra >> ga >> ba >> rd >> gd >> bd >> rs >> gs >> bs >> rr >> gr >> br >> shine;
            STColor3f amb(ra, ga, ba);
            STColor3f diff(rd, gd, bd);
            STColor3f spec(rs, gs, bs);
            STColor3f mirr(rr, gr, br);
            rtMaterial(amb, diff, spec, mirr, shine);
        } else if (command == "TMaterial") {
            float ra, ga, ba, rd, gd, bd, rs, gs, bs, rr, gr, br, shine, rf, gf, bf, snell;
            ss >> ra >> ga >> ba >> rd >> gd >> bd >> rs >> gs >> bs >> rr >> gr >> br >> shine >> rf >> gf >> bf >> snell;
            STColor3f amb(ra, ga, ba);
            STColor3f diff(rd, gd, bd);
            STColor3f spec(rs, gs, bs);
            STColor3f mirr(rr, gr, br);
            STColor3f refr(rf, gf, bf);
            rtTransparentMaterial(amb, diff, spec, mirr, shine, refr, snell);
        } else if (command == "Aperture") {
            float a;
            ss >> a;
            rtSetApeture(a);
        } else if (command == "Focus") {
            float x, y, z;
            ss >> x >> y >> z;
            rtSetFocus(STPoint3(x, y, z));
        }
    }
    sceneFile.close();
}

void Scene::rtLoadTexture(const std::string image_name, int& tex_index)
{
    textures.push_back(new STImage(image_name)); tex_index = textures.size() - 1;
}

void Scene::rtLoadTexture(STImage* texture_image, int& tex_index)
{
    textures.push_back(texture_image); tex_index = textures.size() - 1;
}

void Scene::rtVolumetricTexture(VolumetricTexture* vt)
{
    volumetric_textures.push_back(vt);
}

STColor3f Scene::textureColor(const int texture_index, const STPoint2& uv)
{
    STImage* tex_img = textures[texture_index];
    int i = (int)floor(tex_img->GetWidth()*uv.x);
    int j = (int)floor(tex_img->GetHeight()*uv.y);
    if (i<0)i = 0; if (i>tex_img->GetWidth() - 1)i = tex_img->GetWidth() - 1;
    if (j<0)j = 0; if (j>tex_img->GetHeight() - 1)j = tex_img->GetHeight() - 1;
    STColor3f color(tex_img->GetPixel(i, j));
    return color;
}
