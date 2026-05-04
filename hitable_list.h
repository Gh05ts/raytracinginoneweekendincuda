#ifndef HITABLELISTH
#define HITABLELISTH

#include "hitable.h"

class hitable_list: public hitable  {
    public:
        __device__ hitable_list() : list(NULL), list_size(0), has_cached_box(false) {}
        __device__ hitable_list(hitable **l, int n) : list(l), list_size(n), has_cached_box(false) {
            cache_bounding_box();
        }
        __device__ virtual bool hit(const ray& r, float tmin, float tmax, hit_record& rec) const;
        __device__ virtual bool bounding_box(float t0, float t1, aabb& output_box) const;
        __device__ void cache_bounding_box();
        hitable **list;
        int list_size;
        aabb cached_box;
        bool has_cached_box;
};

__device__ bool hitable_list::hit(const ray& r, float t_min, float t_max, hit_record& rec) const {
        hit_record temp_rec;
        bool hit_anything = false;
        float closest_so_far = t_max;
        for (int i = 0; i < list_size; i++) {
            if (list[i]->hit(r, t_min, closest_so_far, temp_rec)) {
                hit_anything = true;
                closest_so_far = temp_rec.t;
                rec = temp_rec;
            }
        }
        return hit_anything;
}

__device__ void hitable_list::cache_bounding_box() {
    if (list_size < 1) {
        has_cached_box = false;
        return;
    }

    aabb temp_box;
    if (!list[0]->bounding_box(0.0f, 1.0f, temp_box)) {
        has_cached_box = false;
        return;
    }

    cached_box = temp_box;
    for (int i = 1; i < list_size; i++) {
        if (!list[i]->bounding_box(0.0f, 1.0f, temp_box)) {
            has_cached_box = false;
            return;
        }
        cached_box = surrounding_box(cached_box, temp_box);
    }

    has_cached_box = true;
}

__device__ bool hitable_list::bounding_box(float t0, float t1, aabb& output_box) const {
    if (!has_cached_box) return false;
    output_box = cached_box;
    return true;
}

#endif
