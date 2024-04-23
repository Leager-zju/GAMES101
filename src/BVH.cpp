#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (size_t i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (size_t i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        size_t n = objects.size();
        size_t pivot = n/2;
        if (splitMethod == SplitMethod::SAH) {
            size_t nBuckets = 10;

            pivot = 1;
            double minCost = kInfinity;
            for (size_t i = 1; i <= nBuckets; i++) {
                Bounds3 left;
                Bounds3 right;
                size_t p = i*nBuckets;
                for (size_t j = 0; j < p; j++) {
                    left = Union(left, objects[j]->getBounds().Centroid());
                }
                for (size_t j = p; j < n; j++) {
                    right = Union(right, objects[j]->getBounds().Centroid());
                }
                double SLeft = left.SurfaceArea();
                double SRight = right.SurfaceArea();
                double cost = SLeft * i + SRight * (n - i);
                if (cost < minCost) {
                    minCost = cost;
                    pivot = i;
                }
            }
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + pivot;
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    if (node->bounds.IntersectP(
            ray, ray.direction_inv,
            {ray.direction.x < 0, ray.direction.y < 0, ray.direction.z < 0})) {
        if (!node->left && !node->right) {
            return node->object->getIntersection(ray);
        }

        Intersection left  = getIntersection(node->left, ray);
        Intersection right  = getIntersection(node->right, ray);
        if (left.happened && right.happened) {
            return left.distance < right.distance ? left : right;
        }
        if (left.happened) {
            return left;
        }
        if (right.happened) {
            return right;
        }
    }
    return {};
}