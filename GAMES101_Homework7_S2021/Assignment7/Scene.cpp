//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    if (depth > maxDepth)
        return Vector3f(0, 0, 0);

    // find intersection between the incoming ray from camera and the scene
    Intersection xInter = intersect(ray);
    if (!xInter.happened)
        return Vector3f(0, 0, 0);

    Vector3f L_dir(0, 0, 0);  // directly illumination

    Vector3f x = xInter.coords;
    Material *m = xInter.m;
    Vector3f nn = xInter.normal;
    Vector3f ws = ray.direction;

    // caculate the PDF for sampling a point on the light source in the scene uniformly
    Intersection lightPoint;
    float samplePDF = 0.0f;
    sampleLight(lightPoint, samplePDF);

    Vector3f p = lightPoint.coords;
    Vector3f wo = normalize(p - x);
    Vector3f emit = lightPoint.emit;

    // check if the direct ray (reversed) from x to p is blocked
    // Ray dirRay(x, wo);  // maybe blocked by x itself (t_enter == t_exit == 0), add some offset
    Ray dirRay(x + EPSILON * nn, wo);
    Intersection pInter = intersect(dirRay);
    if (pInter.happened && pInter.m->hasEmission()) {
        Vector3f tmp = pInter.coords;
        // TODO: how to check if p and tmp is the same point?
        // if (tmp.norm() > p.norm() - EPSILON && tmp.norm() < p.norm() + EPSILON) {
            Vector3f n = pInter.normal;
            L_dir = emit * m->eval(ws, wo, nn) * dotProduct(-wo, n) * dotProduct(wo, nn) / pow(pInter.distance, 2) / samplePDF;
        // }
    }

    Vector3f L_indir(0, 0, 0);  // indirectly illumination

    float p_RR = get_random_float();
    if (p_RR <= RussianRoulette) {
        // sample a indirect ray from x
        Vector3f wi = m->sample(wo, nn);
        Ray indirRay(x, wi);

        // check if the indirect ray hit a non-emitting object
        Intersection hit = intersect(indirRay);
        if (hit.happened && !hit.m->hasEmission()) {
            // treat the hit point q as a light source, caculate the emit of q recursively
            L_indir = castRay(indirRay, depth + 1) * m->eval(ws, wi, nn) * dotProduct(wi, nn) / m->pdf(ws, wi, nn) / RussianRoulette;
        }
    }

    return m->getEmission() + L_dir + L_indir;
}