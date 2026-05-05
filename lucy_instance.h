#ifndef LUCYINSTANCEH
#define LUCYINSTANCEH

#include "hitable.h"

class lucy_instance : public hitable {
public:
    __device__ lucy_instance(vec3 *v0, vec3 *v1, vec3 *v2, int n, const aabb& local_box,
                             const vec3& t, float s, material *m)
        : tri_v0(v0), tri_v1(v1), tri_v2(v2), tri_count(n), local_bbox(local_box),
          translate(t), scale(s), mat_ptr(m) {}

    __device__ virtual bool hit(const ray& r, float t_min, float t_max, hit_record& rec) const;
    __device__ virtual bool bounding_box(float t0, float t1, aabb& output_box) const;

    vec3 *tri_v0; vec3 *tri_v1; vec3 *tri_v2;
    int tri_count;
    aabb local_bbox;
    vec3 translate;
    float scale;
    material *mat_ptr;
};

__device__ bool lucy_instance::hit(const ray& r, float t_min, float t_max, hit_record& rec) const {
    ray local_ray((r.origin() - translate) / scale, r.direction() / scale);
    bool hit_any = false;
    float closest = t_max;
    for (int i = 0; i < tri_count; i++) {
        vec3 a = tri_v0[i], b = tri_v1[i], c = tri_v2[i];
        vec3 e1 = b - a, e2 = c - a;
        vec3 p = cross(local_ray.direction(), e2);
        float det = dot(e1, p);
        if (fabs(det) < 1e-7f) continue;
        float inv_det = 1.0f / det;
        vec3 tvec = local_ray.origin() - a;
        float u = dot(tvec, p) * inv_det;
        if (u < 0.0f || u > 1.0f) continue;
        vec3 q = cross(tvec, e1);
        float v = dot(local_ray.direction(), q) * inv_det;
        if (v < 0.0f || u + v > 1.0f) continue;
        float t = dot(e2, q) * inv_det;
        if (t <= t_min || t >= closest) continue;
        closest = t;
        rec.t = t;
        rec.p = local_ray.point_at_parameter(t) * scale + translate;
        rec.normal = unit_vector(cross(e1, e2));
        rec.mat_ptr = mat_ptr;
        hit_any = true;
    }
    return hit_any;
}

__device__ bool lucy_instance::bounding_box(float t0, float t1, aabb& output_box) const {
    vec3 mn = local_bbox.min() * scale + translate;
    vec3 mx = local_bbox.max() * scale + translate;
    output_box = aabb(mn, mx);
    return true;
}

#endif
