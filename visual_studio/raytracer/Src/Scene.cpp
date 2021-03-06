#define NOMINMAX
#include <Windows.h>

#include "Scene.h"
#include "SceneObjectTransform.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <memory>
#include <limits>

#define DEFAULT_NUM_RENDER_THREADS 4
#define NUM_PIXEL_LOCKS 1009

Scene::Scene()
    : currBsdf(new Lambertian()), currEmittedPower(0.f),
    /*currMaterial(NULL), currTexIndex(-1), use_shadow(true), use_transparent_shadow(false), attenuation_coefficient(1.f),*/ camera() /*accel_structure(NONE), uniform_grid(NULL),*/
#if THREADED
    , pixelLocks(NUM_PIXEL_LOCKS), brightPixelLocks(NUM_PIXEL_LOCKS), renderThreadPool(), renderThreadsDesired(DEFAULT_NUM_RENDER_THREADS)
#endif
    , saveEveryNPercent(100)
    , aabb_tree(NULL)
{
    rtSampleRate(1);
}

Scene::~Scene()
{
    /*
    if (currMaterial != NULL)delete currMaterial;
    if ((int)textures.size() > 0)for (int i = 0; i < (int)textures.size(); i++)delete textures[i];
    if ((int)volumetric_textures.size() > 0)for (int i = 0; i < (int)volumetric_textures.size(); i++)delete volumetric_textures[i];
    if (camera != NULL)delete camera;
    if ((int)objects.size() > 0)for (int i = 0; i < (int)objects.size(); i++)delete objects[i];
    if ((int)lights.size() > 0)for (int i = 0; i < (int)lights.size(); i++)delete lights[i];
    if ((int)areaLights.size() > 0)for (int i = 0; i < (int)areaLights.size(); i++)delete areaLights[i];
    if ((int)aabb_trees.size() > 0)for (int i = 0; i < (int)aabb_trees.size(); i++)delete aabb_trees[i];
    if (uniform_grid != NULL)delete uniform_grid;
    */
    if ((int)objects.size() > 0)for (int i = 0; i < (int)objects.size(); i++)delete objects[i];
    if (aabb_tree) delete aabb_tree;
}

/*std::string Scene::info() {
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
}*/



float Scene::S_i_at(
#if USE_EIGEN
    const std::vector<Vertex, Eigen::aligned_allocator<Vertex>>& vertices,
#else
    const std::vector<Vertex>& vertices,
#endif
    int i) {

    float Pa_from_i1 = vertices[i].Pa_from_next;
    return S_i_at(vertices, i, Pa_from_i1);
}

float Scene::S_i_at(
#if USE_EIGEN
    const std::vector<Vertex, Eigen::aligned_allocator<Vertex>>& vertices,
#else
    const std::vector<Vertex>& vertices,
#endif
    int i, float Pa_from_i1) {

    float S_1i = (i == 0) ? 0.f : vertices[i - 1].S;
    return S_i_at(vertices, i, Pa_from_i1, S_1i);
}

float Scene::S_i_at(
#if USE_EIGEN
    const std::vector<Vertex, Eigen::aligned_allocator<Vertex>>& vertices,
#else
    const std::vector<Vertex>& vertices,
#endif
    int i, float Pa_from_i1, float S_1i) {

    float pi_over_pi1 = Pa_from_i1 / vertices[i].Pa_from_prev;
    return (pi_over_pi1 * pi_over_pi1) * (vertices[i].prev_gap_nonspecular + S_1i);
}


float Scene::qPsig_a_to_b(const Vertex& a, const Vertex& b, const STVector3& w_ab, const STVector3& w_ac) {
    STColor3f f_a_to_b = a.f(w_ac, w_ab);
    float Psig_a_to_b = a.p_sig(w_ac, w_ab);
    return std::min(f_a_to_b.maxComponent(), Psig_a_to_b);

    // old way: may cause divide-by-zero (Psig_a_to_b might be 0)
    //float q_a_to_b = std::min(f_a_to_b.maxComponent() / Psig_a_to_b, 1.f);
    //return q_a_to_b * Psig_a_to_b;
}





// struct for storing paths of interest for debugging
struct Path {
    int x, y;
    int a, b;
    int s, t;
    std::vector<Vertex> vertices_L;
    std::vector<Vertex> vertices_E;
    STColor3f Cs_st, C_st;
    float w_st;

    Path(int x, int y, int a, int b, int s, int t, const std::vector<Vertex>& vertices_L, const std::vector<Vertex>& vertices_E, float w_st, STColor3f Cs_st, STColor3f C_st)
        : x(x), y(y), a(a), b(b), s(s), t(t), vertices_L(vertices_L), vertices_E(vertices_E), w_st(w_st), Cs_st(Cs_st), C_st(C_st) {
    }

    void writeContributionOnly(std::ostream& os) const {
        os << x << ", " << y << ",  " << a << ", " << b << ",  " << s << ", " << t
            << ",   " << C_st.r << ", " << C_st.g << ", " << C_st.b << std::endl;
    }

    void writeAbridged(std::ostream& os) const {
        os << std::endl;
        os << x << ", " << y << ",  " << a << ", " << b << ",  " << s << ", " << t << std::endl;
        os << "C_st: (" << C_st.r << ", " << C_st.g << ", " << C_st.b << ") = " 
            << w_st << " * ("
            << Cs_st.r << ", " << Cs_st.g << ", " << Cs_st.b << ")" << std::endl;

        os << "(S)";
        for (int i = 0; i < t; i++) {
            os << (vertices_E[i].isSpecular() ? "S" : "D");
        }
        if (s > 0) {
            os << "--";
            for (int i = s - 1; i >= 0; i--) {
                os << (vertices_L[i].isSpecular() ? "S" : "D");
            }
        }
        os << "(D)" << std::endl;
    }

    void write(std::ostream& os) const {
        os << std::endl << std::endl;
        os << "=================================================================================================" << std::endl << std::endl;

        os << x << ", " << y << ",  " << a << ", " << b << ",  " << s << ", " << t << std::endl;
        os << "C_st: (" << C_st.r << ", " << C_st.g << ", " << C_st.b << ") = "
            << w_st << " * ("
            << Cs_st.r << ", " << Cs_st.g << ", " << Cs_st.b << ")" << std::endl;

        for (int i = 0; i < t; i++) {
            os << std::endl;
            os << "z" << i << std::endl;
            writeVertex(os, vertices_E[i]);
        }

        for (int i = s-1; i >= 0; i--) {
            os << std::endl;
            os << "y" << i << std::endl;
            writeVertex(os, vertices_L[i]);
        }
    }

    void writeVertex(std::ostream& os, const Vertex& v) const {
        const Intersection& in = v.getIntersection();
        os << "p: (" << in.point.x() << ", " << in.point.y() << ", " << in.point.z() << ")" << std::endl;
        os << "n: (" << in.normal.x() << ", " << in.normal.y() << ", " << in.normal.z() << ")" << std::endl;
        os << "t_prev: " << in.t << std::endl;
        os << "w_to_prev dot n: " << v.w_to_prev.dot(v.getIntersection().normal) << std::endl;
        os << "----------------" << std::endl;
        os << v.getBsdfDescriptionString() << std::endl;
        os << "prev_gap_nonspecular: " << v.prev_gap_nonspecular << std::endl;
        os << "----------------" << std::endl;
        os << "alpha: " << v.alpha.r << " " << v.alpha.g << " " << v.alpha.b << std::endl;
        os << "G_prev: " << v.G_prev << std::endl;
        os << "qPsig_adj: " << v.qPsig_adj << std::endl;
        os << "Pa_from_prev: " << v.Pa_from_prev << "  Pa_from_next: " << v.Pa_from_next << std::endl;
        os << "S: " << v.S << std::endl;
    }
};


//std::vector<Path> paths;


void Scene::generateEyeSubpath(float u, float v, int x, int y, 
#if USE_EIGEN
    std::vector<Vertex, Eigen::aligned_allocator<Vertex>>& vertices,
#else
    std::vector<Vertex>& vertices,
#endif
    STColor3f* C_0t_sum) {

    camera.setSampleUV(u, v);  // this will tell cameraBsdf which direction w to choose for z0->z1

    STPoint3 z0 = STPoint3(0.f, 0.f, 0.f);
    STVector3 z0_n = STVector3(0.f, 0.f, 0.f);
    const Bsdf* z0_bsdf;
    float Pa_z0;
    STColor3f We0_z0;
    camera.sample_z0(&z0, &z0_n, &z0_bsdf, &Pa_z0, &We0_z0);

    // record z0
    vertices.push_back(Vertex(Intersection(0.f, z0, z0_n), z0_bsdf, (const SceneObject*)NULL));
    //vertices.back().w_to_prev                    // not defined for z0
    vertices.back().alpha = We0_z0 / Pa_z0;
    //vertices.back().G_prev                    // not defined for y0
    //vertices.back().qPsig_adj                 // calculated when next w is chosen
    vertices.back().Pa_from_prev = Pa_z0;
    //vertices.back().Pa_from_next              // calculated when next vertex and w after is chosen
    vertices.back().prev_gap_nonspecular = 0.f; // z0 is nonspecular but z_(-1) is specular
    //vertices.back().S                         // calculated when next vertex and w after is chosen

    
    int i = 0;      // v_i is last inserted vertex
    while (true) {
        // choose next direction w
        STVector3 w = STVector3(0.f, 0.f, 0.f);
        float Psig;
        float cos_sampled_w;
        STColor3f f = vertices[i].sample_f(vertices[i].w_to_prev, &w, &Psig, &cos_sampled_w);

        /*// after the path reaches MIN_SUBPATH_LENGTH + 1 vertices, terminate the path
        // with some probability (1-q) where q = f / Psig for the next direction chosen.
        float q = 1.f;
        if (i >= MIN_SUBPATH_LENGTH) {
            float q = std::min(f.maxComponent() / Psig, 1.f);
            if (randFloat() >= q) {
                return;
            }
        }*/

        // if we do q=1 for all i<MIN_SUBPATH_LENGTH, then q will not have reciprocity.
        // this leads to errors in our pi/pi1 ratios when calculating S sums since
        // q_to_next may differ from q_to_prev depending on how many vertices are to
        // either side of vi in a particular path sample.
        // to prevent this, q will simply be min(f/Psig, 1) at all vertices.
        float q = std::min(f.maxComponent() / Psig, 1.f);
        if (randFloat() >= q) {
            return;
        }

        // find intersection between chosen direction and scene.
        const Intersection& rayFromInter = vertices[i].getIntersection();
        Ray w_ray(rayFromInter.point, w, vertices[i].getObj(), rayFromInter.normal, shadowBias);
        const SceneObject* inter_obj = NULL;
        Intersection inter;
        if (!Intersect(w_ray, &inter_obj, &inter)) {
            // ray didn't hit anything; terminate path
            return;
        }

        // calculate qPsig_adj for v_i, now that we know the direction
        // from it to the next vertex
        vertices[i].qPsig_adj = q * Psig;

        if (i >= 1) {
            // calculate Pa_from_next for vertex v[i-1]
            vertices[i - 1].Pa_from_next = vertices[i].qPsig_adj * vertices[i].G_prev;

            // calculate S for v[i-1]
            vertices[i - 1].S = S_i_at(vertices, i - 1);
        }

        // calculate terms in G
        float r = inter.t;
        float cos_intersected_w = fabsf(-w.dot(inter.normal));

        // record new vertex
        vertices.push_back(Vertex(inter, inter_obj->getBsdf(), inter_obj));
        i++;
        vertices[i].w_to_prev = -w;
        vertices[i].alpha = vertices[i - 1].alpha * (f / vertices[i - 1].qPsig_adj);
        vertices[i].G_prev = cos_sampled_w * cos_intersected_w / (r*r);
        vertices[i].Pa_from_prev = vertices[i - 1].qPsig_adj * vertices[i].G_prev;
        vertices[i].prev_gap_nonspecular =
            (vertices[i].isSpecular() || vertices[i - 1].isSpecular()) ? 0.f : 1.f;

        // --------------------------------------------------------------------------------------------------------------------

        // if we hit a light, accumulate the contribution of the sample path from technique p_0t
        // where t is the number of vertices we have so far.
        // the sample from technique p_0t is only nonzero if the eye-subpath prefix ends on a light

        if (inter_obj->emitsLight()) {
            const STVector3& zi_z1i_w = vertices[i].w_to_prev;
            const STVector3& zi_n = vertices[i].getIntersection().normal;

            // calculate unweighted contribution C*_0t
            STColor3f Cs_0t = vertices[i].alpha * lightDistribution.Le(inter_obj, zi_z1i_w, zi_n);

            // note: we know vertices.size()>=2 at this point, i.e. i>=1

            float qPsig_zi_z1i = lightDistribution.qPsig_y0_y1(inter_obj, zi_z1i_w, zi_n);
            float z1i_Pa_from_next = qPsig_zi_z1i * vertices[i].G_prev;
            float S_1i = S_i_at(vertices, i - 1, z1i_Pa_from_next);

            // calculate S_i
            float zi_Pa_from_next = lightDistribution.Pa_y0(inter_obj); // Pa(zi)
            float S_i = S_i_at(vertices, i, zi_Pa_from_next, S_1i);

            // calculate weight w_0t knowing that S_i = (p1t/pt)^2 + ... + (p0/pt)^2
            float w_0t = 1.f / (1.f + S_i);

            STColor3f C_0t = w_0t * Cs_0t;

            if (S_i == 0.f && i > 2) {
                //paths.emplace_back(curr_x, curr_y, curr_a, curr_b, 0, i + 1, std::vector<Vertex>(), vertices, w_0t, Cs_0t, (w_0t * Cs_0t));
                //continue;
                
                //brightPixels[curr_y * width + curr_x] += (C_0t / (float)(sampleRate * sampleRate));
                if (i == 3) {
                    AddCstToBrightPixel(x, y, C_0t);
                }
                continue;
            }

            *C_0t_sum += C_0t;

            /*if (curr_x == target_x && curr_y == target_y) {
                paths.emplace_back(curr_x, curr_y, curr_a, curr_b, 0, i + 1, std::vector<Vertex>(), vertices, w_0t, Cs_0t, (w_0t * Cs_0t));
            }*/
        }
    }
}


void Scene::generateLightSubpath(
#if USE_EIGEN
    std::vector<Vertex, Eigen::aligned_allocator<Vertex>>& vertices
#else
    std::vector<Vertex>& vertices
#endif
    ) {

    STPoint3 y0 = STPoint3(0.f, 0.f, 0.f);
    STVector3 y0_n = STVector3(0.f, 0.f, 0.f);
    const SceneObject* y0_obj;
    const Bsdf* y0_bsdf;
    float Pa_y0;
    STColor3f Le0_y0;
    lightDistribution.sample_y0(&y0, &y0_n, &y0_obj, &y0_bsdf, &Pa_y0, &Le0_y0);

    // record y0
    vertices.push_back(Vertex(Intersection(0.f, y0, y0_n), y0_bsdf, y0_obj));
    //vertices.back().w_to_prev                    // not defined for y0
    vertices.back().alpha = Le0_y0 / Pa_y0;
    //vertices.back().G_prev                    // not defined for y0
    //vertices.back().qPsig_adj                 // calculated when next w is chosen
    vertices.back().Pa_from_prev = Pa_y0;
    //vertices.back().Pa_from_next              // calculated when next vertex and w after is chosen
    vertices.back().prev_gap_nonspecular = 1.f; // y0, y_(-1) both have nonspecular P
    //vertices.back().S                         // calculated when next vertex and w after is chosen

    
    int i = 0;      // v_i is last inserted vertex
    while (true) {
        // choose next direction w
        STVector3 w = STVector3(0.f, 0.f, 0.f);
        float Psig;
        float cos_sampled_w;
        STColor3f f = vertices[i].sample_f(vertices[i].w_to_prev, &w, &Psig, &cos_sampled_w);

        /*// after the path reaches MIN_SUBPATH_LENGTH + 1 vertices, terminate the path
        // with some probability (1-q) where q = f / Psig for the next direction chosen.
        float q = 1.f;
        if (i >= MIN_SUBPATH_LENGTH ) {
            q = std::min(f.maxComponent() / Psig, 1.f);
            if (randFloat() >= q) {
                return;
            }
        }*/

        // if we do q=1 for all i<MIN_SUBPATH_LENGTH, then q will not have reciprocity.
        // this leads to errors in our pi/pi1 ratios when calculating S sums since
        // q_to_next may differ from q_to_prev depending on how many vertices are to
        // either side of vi in a particular path sample.
        // to prevent this, q will simply be min(f/Psig, 1) at all vertices.
        float q = std::min(f.maxComponent() / Psig, 1.f);
        if (randFloat() >= q) {
            return;
        }

        // find intersection between chosen direction and scene.
        const Intersection& rayFromInter = vertices[i].getIntersection();
        Ray w_ray(rayFromInter.point, w, vertices[i].getObj(), rayFromInter.normal, shadowBias);
        const SceneObject* inter_obj = NULL;
        Intersection inter;
        if (!Intersect(w_ray, &inter_obj, &inter)) {
            // ray didn't hit anything; terminate path
            return;
        }

        // calculate qPsig_adj for v_i, now that we know the direction
        // from it to the next vertex
        vertices[i].qPsig_adj = q * Psig;

        if (i >= 1) {
            // calculate Pa_from_next for vertex v[i-1]
            vertices[i - 1].Pa_from_next = vertices[i].qPsig_adj * vertices[i].G_prev;

            // calculate S for v[i-1]
            vertices[i - 1].S = S_i_at(vertices, i - 1);
        }

        // calculate terms in G
        float r = inter.t;
        float cos_intersected_w = fabsf(-w.dot(inter.normal));

        // record new vertex
        vertices.push_back(Vertex(inter, inter_obj->getBsdf(), inter_obj));
        i++;
        vertices[i].w_to_prev = -w;
        vertices[i].alpha = vertices[i - 1].alpha * (f / vertices[i - 1].qPsig_adj);
        vertices[i].G_prev = cos_sampled_w * cos_intersected_w / (r*r);
        vertices[i].Pa_from_prev = vertices[i - 1].qPsig_adj * vertices[i].G_prev;
        vertices[i].prev_gap_nonspecular =
            (vertices[i].isSpecular() || vertices[i - 1].isSpecular()) ? 0.f : 1.f;
    }
}

void Scene::AddCstToPixel(int x, int y, const STColor3f& C_st) {
    int pixelIndex = y * width + x;
#if THREADED
    std::mutex& pixelLock = pixelLocks[pixelIndex % NUM_PIXEL_LOCKS];
    pixelLock.lock();
    pixels[pixelIndex] += C_st;
    pixelLock.unlock();
#else
    pixels[pixelIndex] += C_st;
#endif
}

void Scene::AddCstToBrightPixel(int x, int y, const STColor3f& C_st) {
    int pixelIndex = y * width + x;
#if THREADED
    std::mutex& pixelLock = brightPixelLocks[pixelIndex % NUM_PIXEL_LOCKS];
    pixelLock.lock();
    brightPixels[pixelIndex] += C_st;
    pixelLock.unlock();
#else
    brightPixels[pixelIndex] += C_st;
#endif
}


void Scene::ProcessPixel(int x, int y) {
    // work on pixel (x, y)
    STColor3f C_sum_this_pixel(0.f);

    // sampleRate^2 stratified estimates (sample-path groups) per pixel
    for (int a = 0; a < sampleRate; a++) {
        for (int b = 0; b < sampleRate; b++) {

            // offset (x,y) by between [a/sampleRate, b/sampleRate] and [(a+1)/sampleRate, (b+1)/sampleRate)]
            // those are the bounds for the (a,b) subpixel within pixel (x,y)
            float xs = (float)x + (((float)a + randFloat()) / sampleRate);
            float ys = (float)y + (((float)b + randFloat()) / sampleRate);

            // convert to screen coords (u,v)
            float u = xs / width;
            float v = ys / height;

#if USE_EIGEN
            std::vector<Vertex, Eigen::aligned_allocator<Vertex>> vertices_E;
            std::vector<Vertex, Eigen::aligned_allocator<Vertex>> vertices_L;
#else
            std::vector<Vertex> vertices_E;
            std::vector<Vertex> vertices_L;
#endif
            // generate eye subpath thru (u,v)
            STColor3f C0t_sum;
            generateEyeSubpath(u, v, x, y, vertices_E, &C0t_sum);

            // generate light subpath
            generateLightSubpath(vertices_L);

            // accumulate C0t contributions
            C_sum_this_pixel += C0t_sum;

            // calculate contributions for all samples created by linking
            // prefixes of eye and light subpaths

            for (size_t t = 1; t <= vertices_E.size(); t++) {
                for (size_t s = 1; s <= vertices_L.size(); s++) {

                    const Vertex& gap_v_E = vertices_E[t - 1];
                    const Vertex& gap_v_L = vertices_L[s - 1];

                    // if either gap vertex is specular, this path will have 0 contribution
                    if (gap_v_E.isSpecular() || gap_v_L.isSpecular()) {
                        continue;
                    }

                    // calculate visibility between gap vertices
                    STPoint3 gap_point_E = gap_v_E.getIntersection().point;
                    STPoint3 gap_point_L = gap_v_L.getIntersection().point;
                    STVector3 gap_EL = gap_point_L - gap_point_E;
                    STVector3 gap_EL_w = gap_EL;
                    gap_EL_w.normalize();

                    STVector3 gap_normal_E = gap_v_E.getIntersection().normal;
                    STVector3 gap_normal_L = gap_v_L.getIntersection().normal;
                    float gap_cosw_E = gap_EL_w.dot(gap_normal_E);
                    float gap_cosw_L = -gap_EL_w.dot(gap_normal_L);

                    // check if gap vector goes out the backface of either vertex
                    if (gap_cosw_E <= 0.f || gap_cosw_L <= 0.f) {
                        continue;
                    }

                    // check if the gap vector intersects anything
                    Ray gap_ray_EL(gap_point_E, gap_EL_w, gap_v_E.getObj(), gap_normal_E,
                        shadowBias, gap_EL.norm() - shadowBias);
                    /*const SceneObject* gap_inter_obj = NULL;
                    Intersection gap_inter;
                    if (Intersect(gap_ray_EL, &gap_inter_obj, &gap_inter)) {
                    continue;
                    }*/
                    if (DoesIntersect(gap_ray_EL)) {
                        continue;
                    }

                    // calculate c_st
                    float G_gap = (gap_cosw_E * gap_cosw_L) / gap_EL.squaredNorm();
                    STColor3f f_gap_E = gap_v_E.f(gap_v_E.w_to_prev, gap_EL_w);
                    STColor3f f_gap_L = gap_v_L.f(gap_v_L.w_to_prev, -gap_EL_w);
                    STColor3f c_st = f_gap_E * G_gap * f_gap_L;

                    // calculate C*st = aEs * c_st * aL (unweighted contribution)
                    STColor3f Cs_st = gap_v_E.alpha * c_st * gap_v_L.alpha;


                    // calculate S at gap vertex E
                    float S_E;
                    float Pa_gap_LtoE = qPsig_a_to_b(gap_v_L, gap_v_E, -gap_EL_w, gap_v_L.w_to_prev)
                        * G_gap;
                    if (t == 1) {
                        S_E = S_i_at(vertices_E, 0, Pa_gap_LtoE);
                    } else {
                        float Pa_E_1E = qPsig_a_to_b(vertices_E[t - 1], vertices_E[t - 2],
                            vertices_E[t - 1].w_to_prev, gap_EL_w)
                            * vertices_E[t - 1].G_prev;

                        float S_1E = S_i_at(vertices_E, t - 2, Pa_E_1E);
                        S_E = S_i_at(vertices_E, t - 1, Pa_gap_LtoE, S_1E);
                    }

                    // calculate S at gap vertex L
                    float S_L;
                    float Pa_gap_EtoL = qPsig_a_to_b(gap_v_E, gap_v_L, gap_EL_w, gap_v_E.w_to_prev)
                        * G_gap;
                    if (s == 1) {
                        S_L = S_i_at(vertices_L, 0, Pa_gap_EtoL);
                    } else {
                        float Pa_L_1L = qPsig_a_to_b(vertices_L[s - 1], vertices_L[s - 2],
                            vertices_L[s - 1].w_to_prev, -gap_EL_w)
                            * vertices_L[s - 1].G_prev;


                        float S_1L = S_i_at(vertices_L, s - 2, Pa_L_1L);
                        S_L = S_i_at(vertices_L, s - 1, Pa_gap_EtoL, S_1L);
                    }

                    // calculate w_st
                    float w_st = 1.f / (S_L + 1.f + S_E);

                    // calculate weighted contribution Cst
                    STColor3f C_st = w_st * Cs_st;

                    // accumulate this sample
                    if (t == 1) {
                        // find where z0->y(s-1) intersects img plane
                        // paths that don't go thru img plane don't contribute
                        float u_w, v_w;
                        camera.getUvOfDirection(gap_EL_w, &u_w, &v_w);
                        if (u_w < 0.f || u_w >= 1.f || v_w < 0.f || v_w >= 1.f) {
                            continue;
                        }
                        // convert to x,y pixel coordinates, accumulate contribution
                        int x_w = (int)(u_w * width);
                        int y_w = (int)(v_w * height);

                        //pixels[y_w * width + x_w] += C_st;
                        AddCstToPixel(x_w, y_w, C_st);

                        /*if (x_w == target_x && y_w == target_y) {
                        paths.emplace_back(curr_x, curr_y, curr_a, curr_b, s, t, vertices_L, vertices_E, w_st, Cs_st, C_st);
                        }*/

                    } else {
                        C_sum_this_pixel += C_st;

                        /*if (curr_x == target_x && curr_y == target_y) {
                        paths.emplace_back(curr_x, curr_y, curr_a, curr_b, s, t, vertices_L, vertices_E, w_st, Cs_st, C_st);
                        }*/
                    }

                } // s loop
            } // t loop

        } // sample-rate loop
    } // sample-rate loop

    //pixels[y * width + x] += C_sum_this_pixel;
    AddCstToPixel(x, y, C_sum_this_pixel);
}






void calculateFromTo(int length, int blocks, int block_i, int* from, int* to) {
    // remainder will be allocated to lower blocks, i.e. if width=100 and blocks_x=6, then
    // blocks will have width 17, 17, 17, 17, 16, 16
    int block_width_nominal = length / blocks;
    int block_width_remainder = length % blocks;

    if (block_i < block_width_remainder) {
        *from = block_i * (block_width_nominal + 1);
        *to = *from + (block_width_nominal + 1);
    } else {
        *from = block_i * block_width_nominal + block_width_remainder;
        *to = *from + block_width_nominal;
    }
}


void Scene::Render() {

    // determine the output filenames
    size_t dot_pos = imageFilename.find_last_of('.');
    std::string extension = imageFilename.substr(dot_pos);
    std::string filenameNoExtension = imageFilename.substr(0, dot_pos);
    if (!(blocks_x == 1 && blocks_y == 1)) {
        filenameNoExtension.append("_")
            .append(std::to_string(blocks_x))
            .append("x")
            .append(std::to_string(blocks_y))
            .append("_")
            .append(std::to_string(block_i))
            .append("_")
            .append(std::to_string(block_j));
    }
    std::string pixelsImgFilename = filenameNoExtension + extension;
    std::string brightPixelsImgFilename = filenameNoExtension + "_bright" + extension;
    std::string sumPixelsImgFilename = filenameNoExtension + "_sum" + extension;



    int x_from, x_to, y_from, y_to;
    calculateFromTo(width, blocks_x, block_i, &x_from, &x_to);
    calculateFromTo(height, blocks_y, block_j, &y_from, &y_to);

   
    lightDistribution.init(objects);

    std::cout << "------------------ray tracing started------------------" << std::endl;


#if THREADED

    renderThreadPool.increaseNumThreads(renderThreadsDesired);

    // schedule render-pixel tasks in 100 batches
    int totalPixels = (x_to - x_from) * (y_to - y_from);
    int pixelsRendered = 0;
    int percent = 0;

    int x = x_from;
    int y = y_from;
    do {
        percent++;
        int pixelsRenderedAfterThisIteration = percent * totalPixels / 100;
        int pixelsToRenderThisIteration = pixelsRenderedAfterThisIteration - pixelsRendered;
        

        const int maxThunksToPush = 100;
        for (int i = 0; i < pixelsToRenderThisIteration; i += maxThunksToPush) {

            int thunksToPush = std::min(maxThunksToPush, pixelsToRenderThisIteration - i);
            for (int j = 0; j < thunksToPush; j++) {

                renderThreadPool.schedule(std::bind(&Scene::ProcessPixel, this, x, y));

                if (x == x_to - 1) {
                    x = x_from;
                    y++;
                } else {
                    x++;
                }
            }
            renderThreadPool.waitUntilNumUnfinishedTasks(renderThreadPool.getNumThreads());
        }
        renderThreadPool.wait();

        pixelsRendered = pixelsRenderedAfterThisIteration;
        std::cout << percent << "% ";

        if (percent % saveEveryNPercent == 0) {
            savePixels(pixelsImgFilename, brightPixelsImgFilename, sumPixelsImgFilename);
        }

    } while (percent < 100);

#else

    int totalPixels = (x_to - x_from) * (y_to - y_from);
    int percent = 0, computed = 0;

    for (int y = y_from; y < y_to; y++) {
        for (int x = x_from; x < x_to; x++) {

            ProcessPixel(x, y);

            computed++;
            if (100 * computed / totalPixels > percent) {
                percent++;
                std::cout << percent << "% ";

                if (percent % saveEveryNPercent == 0) {
                    savePixels(pixelsImgFilename, brightPixelsImgFilename);
                }
            }
        }
    }

#endif

    
    std::cout << "------------------ray tracing finished------------------" << std::endl;



    /*// write stored paths to files (full and abridged printouts)
    if (!paths.empty()) {
        // sort paths by contribution
        std::sort(paths.begin(), paths.end(), [](const Path& a, const Path& b)->bool {
            return a.C_st.maxComponent() > b.C_st.maxComponent();
        });

        std::ofstream ofs(filenameNoExtension + "_paths.txt", std::ofstream::out);
        std::ofstream ofs_ab(filenameNoExtension + "_paths_abr.txt", std::ofstream::out);
        for (const Path& p : paths) {
            //p.writeContributionOnly(ofs_c);
            p.write(ofs);
            p.writeAbridged(ofs_ab);
        }
    }*/
}

void Scene::savePixels(const std::string& pixelsFilename, const std::string& brightPixelsFilename, const std::string& sumPixelsFilename) {

    // each weighted contribution C_st is multiplied by C_ST_MULTIPLIER before being added to a pixel color;
    // it should be proprotional to (sampleRate^2).
    // For any pixel sensor, we're essentially taking (width*height)*sampleRate^2 estimates for its response value.
    // (every pixel response is estimated using the sample paths in these (width*height)*sampleRate^2 estimates,
    // but only a small subset of those paths will contribute to any particular pixel's response).  So, each pixel's
    // response is the sum of the estimates divided by (width*height)*sampleRate^2 to get an avg.  However, as resolution
    // increases, each pixel's sensor must become more sensitive since it's responding to incoming power from a smaller
    // solid angle (since the pixel is smaller), so its sensitivity is proprotional to (width*height).  So our 
    // multiplier should be proprotional to (width*height) / ((width*height)*sampleRate^2) = 1 / (sampleRate^2)

    const float C_ST_MULTIPLIER = 1.f / (float)(sampleRate * sampleRate);

    {
        std::vector<STColor3f> pixelsCopy(pixels);
        const float LOG_SCALE = 1.f;
        for (STColor3f& p : pixelsCopy) {
            p *= C_ST_MULTIPLIER;
            //p = LOG_SCALE * (p + 1.f).Log();
        }
        STImage im(width, height, pixelsCopy);
        im.Save(pixelsFilename);
    }
    {
        std::vector<STColor3f> brightPixelsCopy(brightPixels);
        for (STColor3f& p : brightPixelsCopy) {
            p *= C_ST_MULTIPLIER;
            //p = LOG_SCALE * (p + 1.f).Log();
        }
        STImage im_bright(width, height, brightPixelsCopy);
        im_bright.Save(brightPixelsFilename);
    }
    {
        std::vector<STColor3f> sumPixels(pixels.size());
        for (int i = 0; i < sumPixels.size(); i++) {
            sumPixels[i] = C_ST_MULTIPLIER * (pixels[i] + brightPixels[i]);
        }
        STImage im_sum(width, height, sumPixels);
        im_sum.Save(sumPixelsFilename);
    }
}


/*STColor3f Scene::TraceRay(const Ray &ray, int bounce) {
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
}*/

/*
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
*/

/*
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
*/

/*
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
}*/

void Scene::rtClear()
{
    //currMaterial = NULL;
    matStack.clear();
    matStack.push_back(STTransform4::Identity());
    //focus = 0.;

    currBsdf.reset(new Lambertian());
    currEmittedPower = STColor3f(0.f);
}

void Scene::rtCamera(const STPoint3& eye, const STVector3& up, const STPoint3& lookAt, float fovy, float aspect)
{
    camera.setAttributes(eye, up, lookAt, fovy, aspect);
}

void Scene::rtOutput(int imgWidth, int imgHeight, const std::string& outputFilename)
{
    width = imgWidth;
    height = imgHeight;
    imageFilename = outputFilename;

    pixels.resize(width * height, STColor3f(0.f));
    brightPixels.resize(width * height, STColor3f(0.f));
}

void Scene::rtNumRenderThreads(int n) {
#if THREADED
    renderThreadsDesired = n;
#endif
}

/*void Scene::rtBounceDepth(int depth)
{
    bounceDepth = depth;
}*/

void Scene::rtShadowBias(float bias)
{
    shadowBias = bias;
}

void Scene::rtSampleRate(int r)
{
    sampleRate = r;
    //for (int i = 0; i < (int)areaLights.size(); i++) areaLights[i]->setSampleRate(r);
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
        float conv = M_PI / 180.f;
        STTransform4 M = matStack.back() * RotationMatrix(rx * conv, ry * conv, rz * conv);
        matStack.pop_back();
        matStack.push_back(M);
    }
}

void Scene::rtScale(float sx, float sy, float sz)
{
    if (!matStack.empty()) {
        STTransform4 M = matStack.back() * ScalingMatrix(sx, sy, sz);
        matStack.pop_back();
        matStack.push_back(M);
    }
}

void Scene::rtTranslate(float tx, float ty, float tz)
{
    if (!matStack.empty()) {
        STTransform4 M = matStack.back() * TranslationMatrix(tx, ty, tz);
        matStack.pop_back();
        matStack.push_back(M);
    }
}

void Scene::rtSphere(const STPoint3& center, float radius)
{
    addShapeWithCurrentSceneFileState(new Sphere(center, radius));
}

void Scene::rtTriangle(const STPoint3& v1, const STPoint3& v2, const STPoint3& v3)
{
    addShapeWithCurrentSceneFileState(new Triangle(v1, v2, v3));
}

void Scene::rtQuadAA(int uIdx, bool uPosNormal, float xmin, float xmax, float ymin, float ymax, float zmin, float zmax)
{
    addShapeWithCurrentSceneFileState(new QuadAA(uIdx, uPosNormal, xmin, xmax, ymin, ymax, zmin, zmax));
}

void Scene::rtTriangle(const STPoint3& v1, const STPoint3& v2, const STPoint3& v3, const STPoint2& uv1, const STPoint2& uv2, const STPoint2& uv3)
{
    addShapeWithCurrentSceneFileState(new Triangle(v1, v2, v3, uv1, uv2, uv3));
}

void Scene::rtBox(const STPoint3& o, const STPoint3& x, const STPoint3& y, const STPoint3& z)
{
    addShapeWithCurrentSceneFileState(new Box(o, x, y, z));
}

void Scene::rtBox(const STPoint3& center, const STVector3& size)
{
    addShapeWithCurrentSceneFileState(new Box(center, size));
}

void Scene::rtCylinder(const STPoint3& A, const STPoint3& B, float radius)
{
    addShapeWithCurrentSceneFileState(new Cylinder(A, B, radius));
}

void Scene::rtQJulia(const float4& mu, const float epsilon)
{
    addShapeWithCurrentSceneFileState(new quaternionJuliaSet(mu, epsilon));
}

void Scene::rtSaveEveryNPercent(int n)  {
    saveEveryNPercent = std::max(1, std::min(n, 100));
}

/*void Scene::rtParticipatingMedia(const STPoint3& center, const STVector3& size, const std::string& file_name)
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
        objects.push_back(new SceneObject(new CompoundShape(one->shape, two->shape, c, shadowBias), matStack.back(), newCopyBsdf(&*currBsdf), currEmittedPower));
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
}*/

void Scene::rtTriangleMesh(const std::string& file_name, const bool& counter_clockwise, const bool& smoothed_normal)
{
#if USE_EIGEN
    std::vector<STTriangleMesh, Eigen::aligned_allocator<STTriangleMesh>> meshes;
#else
    std::vector<STTriangleMesh> meshes;
#endif
    STTriangleMesh::LoadObj(meshes, file_name);
    for (int i = 0; i < (int)meshes.size(); i++) {
        addShapeWithCurrentSceneFileState(new TriangleMesh(meshes[i], counter_clockwise, smoothed_normal));
    }
    //objects.push_back(new SceneObject(new TriangleMesh(file_name,counter_clockwise,smoothed_normal), currMaterial, &matStack.back(), currTexIndex));
}

/*void Scene::rtTriangleMeshWithMaterialAndTexture(const std::string& file_name, const bool& counter_clockwise, const bool& smoothed_normal)
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
        Material* mat = new Material(amb, diff, spec, STColor3f(), shin);
        objects.push_back(new SceneObject(new TriangleMesh(*meshes[i], counter_clockwise, smoothed_normal), matStack.back(), newCopyBsdf(&*currBsdf), currEmittedPower));
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
}*/

void Scene::addShapeWithCurrentSceneFileState(Shape* shape) {
    if ((matStack.back() - STTransform4::Identity()).norm() < 0.001f) {
        objects.push_back(new SceneObject(shape, newCopyBsdf(&*currBsdf), currEmittedPower));
    } else {
        objects.push_back(new SceneObjectTransform(shape, matStack.back(), newCopyBsdf(&*currBsdf), currEmittedPower));
    }
}

void Scene::buildAABBTrees()
{
    aabb_tree = new AABBTree(objects);
}

#define USE_ACCEL 0

bool Scene::Intersect(const Ray& ray, SceneObject const** object, Intersection* inter)
{
    /*switch (accel_structure){
    case AABB_TREE:return IntersectAABBTree(ray, object);
    case UNIFORM_GRID:return IntersectUniformGrid(ray, object);
    default:return IntersectionNoAccelStructure(ray, object);
    }*/
#if USE_ACCEL
    return IntersectAABBTree(ray, object, inter);
#else
    return IntersectionNoAccelStructure(ray, object, inter);
#endif
}

bool Scene::DoesIntersect(const Ray& ray) {
#if USE_ACCEL
    return DoesIntersectAABBTree(ray);
#else
    return DoesIntersectNoAccelStructure(ray);
#endif
}

bool Scene::IntersectionNoAccelStructure(const Ray& ray, SceneObject const** object, Intersection* inter)
{
    Intersection min_inter(FLT_MAX, STPoint3(0.f, 0.f, 0.f), STVector3(0.f, 0.f, 0.f));
    const SceneObject* min_object = NULL;
    for (const SceneObject* obj : objects) {
        Intersection inter;
        if (obj->getIntersect(ray, &inter)) {
            if (inter.t < min_inter.t && ray.inRange(inter.t)) {    // inRange should be checked already by obj.getIntersect()
                min_inter = inter;
                min_object = obj;
            }
        }
    }
    *object = min_object;
    *inter = min_inter;
    return (min_object != NULL);
}


bool Scene::DoesIntersectNoAccelStructure(const Ray& ray) {
    for (const SceneObject* obj : objects) {
        if (obj->doesIntersect(ray)) {
            return true;
        }
    }
    return false;
}

bool Scene::IntersectAABBTree(const Ray& ray, SceneObject const** object, Intersection* inter) {
    *object = aabb_tree->getIntersectionWithObject(ray, inter);
    return *object != NULL;
}

bool Scene::DoesIntersectAABBTree(const Ray& ray) {
    return aabb_tree->doesIntersect(ray);
}
/*
Intersection* Scene::IntersectUniformGrid(const Ray& ray, SceneObject*& object)
{
    return uniform_grid->getIntersectionWithObject(ray, object);
}

void Scene::getObjectsAABB(const std::vector<SceneObject*>& objs, AABB& aabb)
{
    aabb.xmin = FLT_MAX; aabb.xmax = -FLT_MAX; aabb.ymin = FLT_MAX; aabb.ymax = -FLT_MAX; aabb.zmin = FLT_MAX; aabb.zmax = -FLT_MAX;
    for (int i = 0; i < (int)objects.size(); i++){
        AABB::combine(&aabb, objects[i]->aabb, &aabb);
    }
    float offset = aabb.maxEdgeLength()*.001f;
    aabb.enlarge(offset);
}*/

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
            STPoint3 eye = STPoint3(ex, ey, ez);
            STVector3 up = STVector3(ux, uy, uz);
            STPoint3 lookAt = STPoint3(lx, ly, lz);
            rtCamera(eye, up, lookAt, f, a);
        } else if (command == "Output") {
            int w, h;
            std::string fname;
            ss >> w >> h >> fname;
            rtOutput(w, h, fname);
        } else if (command == "Threads") {
            int n;
            ss >> n;
            rtNumRenderThreads(n);
        } else if (command == "SaveEveryNPercent") {
            int n;
            ss >> n;
            rtSaveEveryNPercent(n);
        }/*else if (command == "BounceDepth") {
            int depth;
            ss >> depth;
            rtBounceDepth(depth);
        }*/ else if (command == "ShadowBias") {
            float bias;
            ss >> bias;
            rtShadowBias(bias);
        } else if (command == "SampleRate") {
            int rate;
            ss >> rate;
            rtSampleRate(rate);
        } else if (command == "PushMatrix") {
            rtPushMatrix();
        } else if (command == "PopMatrix") {
            rtPopMatrix();
        } else if (command == "LoadMatrix") {
            STTransform4 mat;
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    ss >> mat(i,j);
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
            STPoint3 center = STPoint3(cx, cy, cz);
            rtSphere(center, r);
        } else if (command == "Triangle") {
            float x1, y1, z1, x2, y2, z2, x3, y3, z3;
            ss >> x1 >> y1 >> z1 >> x2 >> y2 >> z2 >> x3 >> y3 >> z3;
            STPoint3 v[3];
            v[0] = STPoint3(x1, y1, z1);
            v[1] = STPoint3(x2, y2, z2);
            v[2] = STPoint3(x3, y3, z3);
            rtTriangle(v[0], v[1], v[2]);
        } else if (command == "QuadAA") {
            char xyz, normalSign;
            float xmin, xmax, ymin, ymax, zmin, zmax;
            ss >> xyz >> normalSign >> xmin >> xmax >> ymin >> ymax >> zmin >> zmax;
            int uIdx = -1;
            switch (xyz) {
            case 'x': uIdx = 0; break;
            case 'y': uIdx = 1; break;
            case 'z': uIdx = 2; break;
            default:
                printf("WARNING! QuadAA constant dimension not 'x', 'y', or 'z'. Using 'x'.\n");
                uIdx = 0;
                break;
            }
            bool uPosNormal;
            switch (normalSign) {
            case '+': uPosNormal = true; break;
            case '-': uPosNormal = false; break;
            default:
                printf("WARNING! QuadAA u-dimension pos/neg normal not '+' or '-'.  Using '+'.\n");
                uPosNormal = true;
                break;
            }
            rtQuadAA(uIdx, uPosNormal, xmin, xmax, ymin, ymax, zmin, zmax);
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
            STPoint3 A = STPoint3(a1, a2, a3);
            STPoint3 B = STPoint3(b1, b2, b3);
            rtCylinder(A, B, r);
        } else if (command == "QJulia") {
            float4 mu;
            float epsilon;
            ss >> mu.x() >> mu.y() >> mu.z() >> mu.w() >> epsilon;
            rtQJulia(mu, epsilon);
        } /*else if (command == "ParticipatingMedia") {
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
        }*/
        else if (command == "TriangleMesh") {
            std::string file_name; int counter_clockwise; int smoothed_normal;
            ss >> file_name >> counter_clockwise >> smoothed_normal;
            rtTriangleMesh(file_name, (counter_clockwise != 0), (smoothed_normal != 0));
        } /*else if (command == "AmbientLight") {
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
        }*/ else if (command == "Material") {
            /*float ra, ga, ba, rd, gd, bd, rs, gs, bs, rr, gr, br, shine;
            ss >> ra >> ga >> ba >> rd >> gd >> bd >> rs >> gs >> bs >> rr >> gr >> br >> shine;
            STColor3f amb(ra, ga, ba);
            STColor3f diff(rd, gd, bd);
            STColor3f spec(rs, gs, bs);
            STColor3f mirr(rr, gr, br);
            rtMaterial(amb, diff, spec, mirr, shine);*/

            ss >> currEmittedPower.r >> currEmittedPower.g >> currEmittedPower.b;

            std::string type;
            ss >> type;
            if (type == "L") {
                STColor3f R;
                ss >> R.r >> R.g >> R.b;
                currBsdf.reset(new Lambertian(R));
            } else if (type == "SC") {
                STColor3f R, eta, k;
                ss >> R.r >> R.g >> R.b;
                ss >> eta.r >> eta.g >> eta.b;
                ss >> k.r >> k.g >> k.b;
                currBsdf.reset(new SpecularCond(R, eta, k));
            } else if (type == "SD") {
                STColor3f R, T;
                float etai, etat;
                ss >> R.r >> R.g >> R.b;
                ss >> T.r >> T.g >> T.b;
                ss >> etai >> etat;
                currBsdf.reset(new SpecularDiel(R, T, etai, etat));
            }
        }/* else if (command == "TMaterial") {
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
        }*/
    }
    sceneFile.close();
}

/*void Scene::rtLoadTexture(const std::string image_name, int& tex_index)
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
}*/

void Scene::setRenderSubimage(int blocks_x, int blocks_y, int block_i, int block_j) {
    this->blocks_x = blocks_x;
    this->blocks_y = blocks_y;
    this->block_i = block_i;
    this->block_j = block_j;
}
