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

__device__ inline float aabb_surface_area(const aabb &box) {
    vec3 d = box.max() - box.min();
    return 2.0f * (d.x() * d.y() + d.x() * d.z() + d.y() * d.z());
}

__device__ inline bool build_range_box(hitable **l, int start, int end, aabb &out_box) {
    if (end <= start) return false;
    aabb temp;
    if (!l[start]->bounding_box(0.0f, 1.0f, out_box)) return false;
    for (int i = start + 1; i < end; i++) {
        if (!l[i]->bounding_box(0.0f, 1.0f, temp)) return false;
        out_box = surrounding_box(out_box, temp);
    }
    return true;
}

__device__ int choose_sah_split(hitable **l, int start, int end) {
    int object_span = end - start;
    if (object_span <= 2) return start + 1;

    float best_cost = 1e30f;
    int best_axis = 0;
    int best_mid = choose_sah_split(l, start, end); 
    // start + object_span / 2;

    for (int axis = 0; axis < 3; axis++) {
        sort_hitable_by_axis(l, start, end, axis);
        for (int mid = start + 1; mid <= end - 1; mid++) {
            aabb left_box, right_box;
            if (!build_range_box(l, start, mid, left_box) || !build_range_box(l, mid, end, right_box)) {
                continue;
            }
            float left_area = aabb_surface_area(left_box);
            float right_area = aabb_surface_area(right_box);
            int left_count = mid - start;
            int right_count = end - mid;
            float cost = left_area * left_count + right_area * right_count;
            if (cost < best_cost) {
                best_cost = cost;
                best_axis = axis;
                best_mid = mid;
            }
        }
    }

    sort_hitable_by_axis(l, start, end, best_axis);
    return best_mid;
}

class bvh_node : public hitable {
public:
    bool use_sah_split;
    __device__ bvh_node(): left_node(NULL), right_node(NULL), left_obj(NULL), right_obj(NULL), is_leaf(false), use_sah_split(true) {}
    __device__ bvh_node(hitable **l, int n, curandState *local_rand_state, bool use_sah): bvh_node(l, 0, n, local_rand_state, use_sah) {}
    __device__ bvh_node(hitable **l, int start, int end, curandState *local_rand_state) : bvh_node(l, start, end, local_rand_state, true) {}

    __device__ bvh_node(hitable **l, int start, int end, curandState *local_rand_state, bool use_sah)
        : left_node(NULL), right_node(NULL), left_obj(NULL), right_obj(NULL), is_leaf(false), use_sah_split(use_sah) {
        int axis = int(3 * curand_uniform(local_rand_state));
        sort_hitable_by_axis(l, start, end, axis);

        int object_span = end - start;
        if (object_span <= 2) {
            is_leaf = true;
            left_obj = l[start];
            right_obj = (object_span == 2) ? l[start + 1] : l[start];
        } else {
            int mid = start + object_span / 2;
            if (use_sah_split) {
                mid = choose_sah_split(l, start, end);
            } else {
                aabb node_box;
                build_range_box(l, start, end, node_box);
                vec3 ext = node_box.max() - node_box.min();
                int axis = (ext.x() > ext.y() && ext.x() > ext.z()) ? 0 : ((ext.y() > ext.z()) ? 1 : 2);
                sort_hitable_by_axis(l, start, end, axis);
            }
            left_node = new bvh_node(l, start, mid, local_rand_state, use_sah_split);
            right_node = new bvh_node(l, mid, end, local_rand_state, use_sah_split);
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
