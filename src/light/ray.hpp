#ifndef RAY_HPP_
#define RAY_HPP_

#include "vec.hpp"
#include "common.hpp"
#include <algorithm>
#include <limits>

// Axis-aligned bounding box used for cheap early-reject tests before
// running the full (more expensive) object intersection routine.
struct AABB {
  Vec3f min;
  Vec3f max;

  // Empty by default (min > max). IntersectsAABB/PointInAABB treat an
  // empty box as "always intersects" so objects whose bounds were never
  // set are never wrongly rejected.
  AABB()
      : min(std::numeric_limits<float>::infinity(),
            std::numeric_limits<float>::infinity(),
            std::numeric_limits<float>::infinity()),
        max(-std::numeric_limits<float>::infinity(),
            -std::numeric_limits<float>::infinity(),
            -std::numeric_limits<float>::infinity()) {}

  void Expand(const Vec3f& p) {
    min.x = std::min(min.x, p.x);
    min.y = std::min(min.y, p.y);
    min.z = std::min(min.z, p.z);
    max.x = std::max(max.x, p.x);
    max.y = std::max(max.y, p.y);
    max.z = std::max(max.z, p.z);
  }

  void Inflate(float e) {
    min.x -= e;
    min.y -= e;
    min.z -= e;
    max.x += e;
    max.y += e;
    max.z += e;
  }
};

// light ray modelled parametrically as r(t) = origin + dir * t
// , where dir = unit(target - origin) is the normalized direction
// => given an object at `target`, at distance d = ||target-origin||
// t = 0 maps to origin
// t = d maps to the object
struct Ray {
    Ray(const Vec3f &from, const Vec3f &to) {
      dir = (to - from).Unit();
      origin = from;
    }
    Vec3f At(float t) const { return origin + dir*t; }
    Vec3f origin;
    Vec3f dir;
};

// Conservative ray/AABB slab test. Returns false only when the ray can
// definitely not hit anything inside the box; an empty box always
// returns true (no culling).
inline bool IntersectsAABB(const Ray& ray, const AABB& box) {
  if (box.max.x < box.min.x || box.max.y < box.min.y ||
      box.max.z < box.min.z)
    return true;

  const float o[3] = {ray.origin.x, ray.origin.y, ray.origin.z};
  const float d[3] = {ray.dir.x, ray.dir.y, ray.dir.z};
  const float mn[3] = {box.min.x, box.min.y, box.min.z};
  const float mx[3] = {box.max.x, box.max.y, box.max.z};

  float tmin = -std::numeric_limits<float>::infinity();
  float tmax = std::numeric_limits<float>::infinity();
  for (int i = 0; i < 3; ++i) {
    if (std::abs(d[i]) < eps) {
      if (o[i] < mn[i] || o[i] > mx[i]) return false;
      continue;
    }
    const float inv_d = 1.0f / d[i];
    float t1 = (mn[i] - o[i]) * inv_d;
    float t2 = (mx[i] - o[i]) * inv_d;
    if (t1 > t2) std::swap(t1, t2);
    tmin = std::max(tmin, t1);
    tmax = std::min(tmax, t2);
    if (tmin > tmax) return false;
  }
  return tmax > 0.0f;
}

// Conservative point-in-box test used before IsInside() probes; an
// empty box returns true (no culling).
inline bool PointInAABB(const AABB& box, const Vec3f& p) {
  if (box.max.x < box.min.x || box.max.y < box.min.y ||
      box.max.z < box.min.z)
    return true;
  return p.x >= box.min.x && p.x <= box.max.x &&
         p.y >= box.min.y && p.y <= box.max.y &&
         p.z >= box.min.z && p.z <= box.max.z;
}

#endif // RAY_HPP_
