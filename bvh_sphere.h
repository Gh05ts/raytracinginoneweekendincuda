#ifndef BVH_SPHEREH
#define BVH_SPHEREH

#include <curand_kernel.h>
#include "bvh_util.h"

class bvh_sphere_node : public hitable {
public:
    __device__ bvh_sphere_node() : left_node(NULL), right_node(NULL), left_obj(NULL), right_obj(NULL), is_leaf(false) {}
    __device__ bvh_sphere_node(hitable **l, int n, curandState *local_rand_state)
        : bvh_sphere_node(l, 0, n, local_rand_state) {}

    __device__ bvh_sphere_node(hitable **l, int start, int end, curandState *local_rand_state)
        : left_node(NULL), right_node(NULL), left_obj(NULL), right_obj(NULL), is_leaf(false) {
        int axis = int(3 * curand_uniform(local_rand_state));
        sort_hitable_by_axis(l, start, end, axis);

        int object_span = end - start;
        if (object_span <= 2) {
            is_leaf = true;
            left_obj = l[start];
            right_obj = (object_span == 2) ? l[start + 1] : l[start];
        } else {
            int mid = start + object_span / 2;
            left_node = new bvh_sphere_node(l, start, mid, local_rand_state);
            right_node = new bvh_sphere_node(l, mid, end, local_rand_state);
        }

        aabb box_left;
        aabb box_right;
        bool has_left = false;
        bool has_right = false;

        if (is_leaf) {
            has_left = left_obj->bounding_box(0.0f, 1.0f, box_left);
            has_right = right_obj->bounding_box(0.0f, 1.0f, box_right);
        } else {
            has_left = left_node->bounding_box(0.0f, 1.0f, box_left);
            has_right = right_node->bounding_box(0.0f, 1.0f, box_right);
        }

        box = (has_left && has_right) ? surrounding_box(box_left, box_right) : aabb(vec3(0, 0, 0), vec3(0, 0, 0));
    }

    __device__ virtual bool hit(const ray& r, float t_min, float t_max, hit_record& rec) const {
        const int MAX_STACK = 64;
        const bvh_sphere_node *stack[MAX_STACK];
        int sp = 0;
        stack[sp++] = this;

        bool hit_anything = false;
        float closest = t_max;
        hit_record temp_rec;

        while (sp > 0) {
            const bvh_sphere_node *node = stack[--sp];
            if (!node->box.hit(r, t_min, closest)) {
                continue;
            }

            if (node->is_leaf) {
                if (node->left_obj->hit(r, t_min, closest, temp_rec)) {
                    hit_anything = true;
                    closest = temp_rec.t;
                    rec = temp_rec;
                }
                if (node->right_obj->hit(r, t_min, closest, temp_rec)) {
                    hit_anything = true;
                    closest = temp_rec.t;
                    rec = temp_rec;
                }
                continue;
            }

            if (sp + 2 <= MAX_STACK) {
                stack[sp++] = node->left_node;
                stack[sp++] = node->right_node;
            }
        }

        return hit_anything;
    }

    __device__ virtual bool bounding_box(float t0, float t1, aabb& output_box) const {
        output_box = box;
        return true;
    }

    bvh_sphere_node *left_node;
    bvh_sphere_node *right_node;
    hitable *left_obj;
    hitable *right_obj;
    bool is_leaf;
    aabb box;
};

#endif