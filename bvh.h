#ifndef BVHH
#define BVHH

#include <curand_kernel.h>
#include "hitable.h"

__device__ inline void swap_hitable(hitable **a, hitable **b) {
    hitable *tmp = *a;
    *a = *b;
    *b = tmp;
}

__device__ inline float box_axis_min(hitable *h, int axis) {
    aabb box;
    if (!h->bounding_box(0.0f, 1.0f, box)) return 0.0f;
    return box.min()[axis];
}

__device__ void sort_hitable_by_axis(hitable **l, int start, int end, int axis) {
    for (int i = start; i < end; i++) {
        for (int j = i + 1; j < end; j++) {
            if (box_axis_min(l[i], axis) > box_axis_min(l[j], axis)) {
                swap_hitable(&l[i], &l[j]);
            }
        }
    }
}

class bvh_node : public hitable {
public:
    __device__ bvh_node() : left_node(NULL), right_node(NULL), left_obj(NULL), right_obj(NULL), is_leaf(false) {}
    __device__ bvh_node(hitable **l, int n, curandState *local_rand_state)
        : bvh_node(l, 0, n, local_rand_state) {}

    __device__ bvh_node(hitable **l, int start, int end, curandState *local_rand_state)
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
            left_node = new bvh_node(l, start, mid, local_rand_state);
            right_node = new bvh_node(l, mid, end, local_rand_state);
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
        const bvh_node *stack[MAX_STACK];
        int sp = 0;
        stack[sp++] = this;

        bool hit_anything = false;
        float closest = t_max;
        hit_record temp_rec;

        while (sp > 0) {
            const bvh_node *node = stack[--sp];
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

    bvh_node *left_node;
    bvh_node *right_node;
    hitable *left_obj;
    hitable *right_obj;
    bool is_leaf;
    aabb box;
};

#endif
