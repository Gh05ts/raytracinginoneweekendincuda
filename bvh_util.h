#ifndef BVH_UTIL_H
#define BVH_UTIL_H

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

__device__ inline void sort_hitable_by_axis(hitable **l, int start, int end, int axis) {
    for (int i = start; i < end; i++) {
        for (int j = i + 1; j < end; j++) {
            if (box_axis_min(l[i], axis) > box_axis_min(l[j], axis)) {
                swap_hitable(&l[i], &l[j]);
            }
        }
    }
}

#endif