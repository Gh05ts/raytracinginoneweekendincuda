#ifndef TRIANGLEH
#define TRIANGLEH

#include "hitable.h"

class triangle : public hitable {
public:
    __device__ triangle() {}
    __device__ triangle(const vec3& a, const vec3& b, const vec3& c, material *m)
        : v0(a), v1(b), v2(c), mat_ptr(m) {}

    __device__ virtual bool hit(const ray& r, float t_min, float t_max, hit_record& rec) const;
    __device__ virtual bool bounding_box(float t0, float t1, aabb& output_box) const;

    vec3 v0, v1, v2;
    material *mat_ptr;
};

__device__ bool triangle::hit(const ray& r, float t_min, float t_max, hit_record& rec) const {
    const float eps = 1e-7f;
    vec3 e1 = v1 - v0;
    vec3 e2 = v2 - v0;
    vec3 p = cross(r.direction(), e2);
    float det = dot(e1, p);
    if (fabs(det) < eps) return false;
    float inv_det = 1.0f / det;
    vec3 tvec = r.origin() - v0;
    float u = dot(tvec, p) * inv_det;
    if (u < 0.0f || u > 1.0f) return false;
    vec3 q = cross(tvec, e1);
    float v = dot(r.direction(), q) * inv_det;
    if (v < 0.0f || u + v > 1.0f) return false;
    float t = dot(e2, q) * inv_det;
    if (t < t_min || t > t_max) return false;
    rec.t = t;
    rec.p = r.point_at_parameter(t);
    rec.normal = unit_vector(cross(e1, e2));
    rec.mat_ptr = mat_ptr;
    return true;
}

__device__ bool triangle::bounding_box(float t0, float t1, aabb& output_box) const {
    const float pad = 1e-4f;
    vec3 small(fminf(v0.x(), fminf(v1.x(), v2.x())) - pad,
               fminf(v0.y(), fminf(v1.y(), v2.y())) - pad,
               fminf(v0.z(), fminf(v1.z(), v2.z())) - pad);
    vec3 big(fmaxf(v0.x(), fmaxf(v1.x(), v2.x())) + pad,
             fmaxf(v0.y(), fmaxf(v1.y(), v2.y())) + pad,
             fmaxf(v0.z(), fmaxf(v1.z(), v2.z())) + pad);
    output_box = aabb(small, big);
    return true;
}

#endif
