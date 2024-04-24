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
    Intersection intersection = intersect(ray);
    if (!intersection.happened) {
        return {0.0, 0.0, 0.0};
    }

    Material *m = intersection.m;
    Vector3f hitPoint = intersection.coords;
    Vector3f N = intersection.normal;
    
    // 打到自发光物体
    // 如果是首次打到，说明是相机调用的 castRay()，直接返回其颜色；
    // 反之，说明是为了计算间接光照项，此时不能对自发光物体采样，返回空值；
    if (m->hasEmission()) {
        return depth == 0 ? m->getEmission() : Vector3f{0.0, 0.0, 0.0};
    }

    switch (m->getType()) {
        case DIFFUSE:
        {
            Vector3f L_dir;
            Vector3f L_indir;
            Vector3f wo = -ray.direction;
        
            {   // 直接光照
                Intersection light;
                float pdf;
                sampleLight(light, pdf);
                float dis = (hitPoint - light.coords).norm();
                
                Vector3f wi = (hitPoint - light.coords).normalized(); // 自发光物体打来的直接光照
                Intersection block = intersect(Ray(hitPoint, -wi));   // 判断该光照是否被其它物体阻挡
                if (block.happened && dis - block.distance < EPSILON) {
                    float dis2 = dis*dis;
                    Vector3f emit = light.emit;
                    Vector3f eval = m->eval(wi, wo, N);
                    float cosTheta = fmax(0.f, -dotProduct(wi, N));
                    float cosThetaPrime = fmax(0.f, dotProduct(wi, light.normal));
                    
                    L_dir = emit * eval * cosTheta * cosThetaPrime / dis2 / pdf;
                }
            }
            {   // 间接光照
                if (get_random_float() < RussianRoulette) {
                    Vector3f sampleDir = m->sample(ray.direction, N).normalized();
                    Vector3f wi = -sampleDir;
                    Vector3f eval = m->eval(wi, wo, N);
                    Vector3f Li = castRay(Ray(hitPoint, sampleDir), depth + 1);
                    float cosTheta = fmax(0.f, dotProduct(sampleDir, N));
                    float pdf = m->pdf(wi, wo, N);

                    L_indir = Li * eval * cosTheta / pdf / RussianRoulette;
                }
            }
            return L_dir + L_indir;
        }
        default:
            break;
    }
    return this->backgroundColor;
}