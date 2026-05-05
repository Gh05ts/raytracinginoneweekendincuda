#ifndef AABBH
#define AABBH

#include "ray.h"

class aabb {
public:
    __device__ aabb() {}
    __device__ aabb(const vec3& a, const vec3& b) : _min(a), _max(b) {}

    __device__ vec3 min() const { return _min; }
    __device__ vec3 max() const { return _max; }

    __device__ bool hit(const ray& r, float t_min, float t_max) const {
        for (int a = 0; a < 3; a++) {
            float invD = 1.0f / r.direction()[a];
            float t0 = (min()[a] - r.origin()[a]) * invD;
            float t1 = (max()[a] - r.origin()[a]) * invD;
            if (invD < 0.0f) {
                float tmp = t0;
                t0 = t1;
                t1 = tmp;
            }
            t_min = t0 > t_min ? t0 : t_min;
            t_max = t1 < t_max ? t1 : t_max;
            if (t_max <= t_min) {
                return false;
            }
        }
        return true;
    }

    vec3 _min;
    vec3 _max;
};

__device__ aabb surrounding_box(aabb box0, aabb box1) {
    vec3 small(fminf(box0.min().x(), box1.min().x()),
               fminf(box0.min().y(), box1.min().y()),
               fminf(box0.min().z(), box1.min().z()));
    
    vec3 big(fmaxf(box0.max().x(), box1.max().x()),
             fmaxf(box0.max().y(), box1.max().y()),
             fmaxf(box0.max().z(), box1.max().z()));
    return aabb(small, big);
}

#endif
