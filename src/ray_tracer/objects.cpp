#include "objects.hpp"

Triangle::Triangle(const Vec3f& a, const Vec3f& b, const Vec3f& c,
                   const Material& mat) : v0(a), v1(b), v2(c) {
  center = (v0 + v1 + v2) / 3.0f;
  material = mat;
}

Vec3f Triangle::NormalAt(const Vec3f& /*at*/) const {
  // constant normal across the surface
  return (v1 - v0).Cross(v2 - v0).Unit();
}

HitRecord Triangle::Intersects(const Ray& ray) const {
  // Moller-Trumbore ray-triangle intersection algorithm
  // notation: TODO link my tutorial
  // ref: web.engr.oregonstate.edu/~mjb/vulkan/Handouts/RayTriangleIntersection.1pp.pdf
  Vec3f edge1 = v1 - v0;
  Vec3f edge2 = v2 - v0;
  // solve the system [edge1 | edge2 | -ray.dir]^T[u | v | t] = 
  //                  [ray.origin | -V0] 
  // using Cramer's method and the properties of mixed product
  Vec3f h = ray.dir.Cross(edge2);
  float a = edge1.Dot(h);
  if (std::abs(a) < eps)
    return {}; // ray is parallel to triangle

  float ainv = 1/a;
  Vec3f s = ray.origin - v0;
  // barycetric coordinate 0..1 of a point along edge 1
  float u = ainv * s.Dot(h);
  if (u < 0 || u > 1)
    return {};

  Vec3f q = s.Cross(edge1);
  // barycetric coordinate 0..1 of a point along edge 2
  float v = ainv * ray.dir.Dot(q);
  if (v < 0 || u + v > 1)
    return {};
  float t = ainv * edge2.Dot(q);
  if (t > 0) { // ray intersection
    return {ray.At(t), true, t};
  } else { // intersection lies behind the ray's origin 
    return {};
  }
}

Quad::Quad(const Vec3f& a, const Vec3f& b, const Vec3f& c, const Vec3f& d,
           const Material& mat) : v0(a), v1(b), v2(c), v3(d) {
  center = (v0 + v1 + v2 + v3) / 4.0f;
  material = mat;
}

Vec3f Quad::NormalAt(const Vec3f& /*at*/) const {
  return (v1 - v0).Cross(v2 - v0).Unit();
}

HitRecord Quad::Intersects(const Ray& ray) const {
  std::array<HitRecord, 2> hits;
  Triangle t1{v0, v1, v2};
  Triangle t2{v0, v2, v3};

  HitRecord h0 = t1.Intersects(ray);
  HitRecord h1 = t2.Intersects(ray);
  HitRecord result;
  if (h0.is_hit && h1.is_hit)
    result = (h0.t < h1.t ? h0 : h1);
  else if (h0.is_hit) 
    result = h0;
  else if (h1.is_hit) 
    result = h1;
  return result;
}

Sphere::Sphere(const Vec3f& c, float r, const Material& mat) {
  center = c;
  radius = r;
  material = mat;
}

// assuming it's on the sphere
Vec3f Sphere::NormalAt(const Vec3f &at) const {
  return (at - center).Unit();
}

bool Sphere::IsInside(const Vec3f &point) const {
  return (point - center).Dot(point - center) < radius * radius;
}

void Sphere::SetTexture(const std::shared_ptr<Image>& tex,
                        int repeat_u,
                        int repeat_v) {
  texture = tex;
  this->repeat_u = std::max(1, repeat_u);
  this->repeat_v = std::max(1, repeat_v);
}

Vec3u8 Sphere::SampleColor(const Vec3f& at) const {
  if (!texture)
    return material.color;
  // compute normal in object space (sphere centered at center)
  Vec3f N = (at - center).Unit();
  // spherical angles - latitude (theta) and longitude (phi)
  float theta = std::atan2(N.z, N.x); // [-pi, pi]
  float phi = std::acos(N.y);         // [0, pi] assuming N normalized
  // equirectangular mapping but for spherical coords
  float u = theta / (2.0f * static_cast<float>(M_PI)) + 0.5f;
  float v = phi / static_cast<float>(M_PI);
  // flip so v=0 is top row in image coordinates
  v = 1.0f - v;
  // apply repeats and wrapping
  u = std::fmod(u * repeat_u, 1.0f);
  v = std::fmod(v * repeat_v, 1.0f);
  if (u < 0) u += 1.0f;
  if (v < 0) v += 1.0f;
  unsigned tw = texture->width, th = texture->height;
  unsigned x = std::min(tw ? tw - 1 : 0, static_cast<unsigned>(u * tw));
  unsigned y = std::min(th ? th - 1 : 0, static_cast<unsigned>(v * th));
  return texture->at(y, x);
}

HitRecord Sphere::Intersects(const Ray& ray) const {
  HitRecord ret; // empty by default (no intersection)
  // notation of ray-sphere intersection formula
  auto C = center;
  auto r = radius;
  auto L = ray.origin - C;
  float a = ray.dir.Dot(ray.dir);
  float b = 2 * ray.dir.Dot(L);
  float c = L.Dot(L) - r * r;

  float discriminant = b*b - 4*a*c;
  ret.is_hit = discriminant > 0;
  if (!ret.is_hit)
    return ret;

  float sqrt_disc = std::sqrt(discriminant);
  float t1 = (-b - sqrt_disc) / (2 * a);
  float t2 = (-b + sqrt_disc) / (2 * a);

  // nearest positive intersection - both must be in front of the ray
  ret.is_hit |= (t1 > 0) || (t2 > 0);
  if (!ret.is_hit)
    return ret;
  float tmin = std::numeric_limits<float>::infinity();
  ret.t = (t1 > 0 && t2 > 0) ? std::min(t1, t2) :
          (t1 > 0 ? t1 :
          (t2 > 0 ? t2 : tmin));
  ret.where = ray.At(ret.t);
  return ret;
}

Block::Block(Vec3f center, float half_x, float half_y, float half_z,
             const Mat3x3& mrot)
  : Block(center, half_x, half_y, half_z, mrot, Material{}) {}

Block::Block(Vec3f center, float half_x, float half_y, float half_z,
             const Mat3x3& mrot,
             const Material& mat) : half_w(half_x),
                                    half_h(half_y),
                                    half_d(half_z),
                                    rot(mrot) {
  this->center = center;
  // CCW normalized vertices
  const int v[8][3] = {
    {-1, -1, -1}, { 1, -1, -1}, { 1,  1, -1}, {-1,  1, -1},
    {-1, -1,  1}, { 1, -1,  1}, { 1,  1,  1}, {-1,  1,  1},
  };
  for (std::size_t i{}; i < 8; ++i) {
    Vec3f vscaled(v[i][0] * half_x, v[i][1] * half_y, v[i][2] * half_z);
    vertices[i] = center + rot * vscaled;
  }
  axisx = (rot * Vec3f{1,0,0}).Unit();
  axisy = (rot * Vec3f{0,1,0}).Unit();
  axisz = (rot * Vec3f{0,0,1}).Unit();
  material = mat;
}

Vec3f Block::NormalAt(const Vec3f& at) const {
  // align with origin and unrotate 
  Vec3f local = rot.Transpose() * (at - center);
  // determine the face: find axis with maximum normalized absolute distance
  float ax = std::abs(local.x) / half_w;
  float ay = std::abs(local.y) / half_h;
  float az = std::abs(local.z) / half_d;

  Vec3f normal_local;
  if (ax >= ay && ax >= az) {
      // closest to +-x face
      float x = (local.x >= 0.f) ? 1.f : -1.f;
      normal_local = Vec3f{x, 0.f, 0.f};
  } else if (ay >= ax && ay >= az) {
      // closest to +-y face
      float y = (local.y >= 0.f) ? 1.f : -1.f;
      normal_local = Vec3f{0.f, y, 0.f};
  } else {
      // closest to +-z face
      float z = (local.z >= 0.f) ? 1.f : -1.f;
      normal_local = Vec3f{0.f, 0.f, z};
  }
  // transform back into world coords 
  return (rot * normal_local).Unit();
}

bool Block::IsInside(const Vec3f& point) const {
  // convert to block-local coordinates
  Vec3f local = rot.Transpose() * (point - center);

  return (std::abs(local.x) <= half_w + eps) &&
         (std::abs(local.y) <= half_h + eps) &&
         (std::abs(local.z) <= half_d + eps);
}

HitRecord Block::Intersects(const Ray& ray) const {
  /* Ray/box intersection slab method instead of checking each
   * face individually. Explanation:
   * https://www.cs.cornell.edu/courses/cs4620/2013fa/lectures/03raytracing1.pdf
   */
  const Mat3x3 rot_t = rot.Transpose();
  const Vec3f o = rot_t * (ray.origin - center);
  const Vec3f d = rot_t * ray.dir;

  const float oa[3] = {o.x, o.y, o.z};
  const float da[3] = {d.x, d.y, d.z};
  const float half[3] = {half_w, half_h, half_d};

  float tmin = -std::numeric_limits<float>::infinity();
  float tmax = std::numeric_limits<float>::infinity();
  for (int i = 0; i < 3; ++i) {
    if (std::abs(da[i]) < eps) {
      // ray is parallel to this slab - it can only hit if already inside
      if (oa[i] < -half[i] || oa[i] > half[i]) return {};
      continue;
    }
    const float inv_d = 1.0f / da[i];
    float t1 = (-half[i] - oa[i]) * inv_d;
    float t2 = (half[i] - oa[i]) * inv_d;
    if (t1 > t2) std::swap(t1, t2);
    tmin = std::max(tmin, t1);
    tmax = std::min(tmax, t2);
    if (tmin > tmax) return {};
  }
  // nearest positive intersection with the box surface
  const float t = tmin > 0.0f ? tmin : tmax;
  if (t <= 0.0f) return {};

  HitRecord ret;
  ret.is_hit = true;
  ret.t = t;
  ret.where = ray.At(t);
  return ret;
}

void Block::SetTexture(const std::shared_ptr<Image>& tex,
                       int repeatx,
                       int repeaty) {
  texture = tex;
  this->repeatx = std::max(1, repeatx);
  this->repeaty = std::max(1, repeaty);
}

Vec3u8 Block::SampleColor(const Vec3f& at) const {
  if (!texture)
    return material.color;
  // at the origin and unrotated 
  Vec3f local = rot.Transpose() * (at - center);
  // find dominant orientation (axis) to identify face
  // the maximum of these distances corresponds to face that was hit
  float ax = std::abs(local.x) / half_w;
  float ay = std::abs(local.y) / half_h;
  float az = std::abs(local.z) / half_d;
  // texture normalized coordinates - they'll later map to a pixel
  float u = 0, v = 0;
  if (ax >= ay && ax >= az) { // +-X faces -> use (z,y)
    u = (local.z / (2*half_d)) + 0.5f;
    v = (local.y / (2*half_h)) + 0.5f;
    if (local.x < 0)
      u = 1 - u; // flip for -x to keep continuity
  } else if (ay >= ax && ay >= az) { // +-Y faces -> use (x,z)
    u = (local.x / (2*half_w)) + 0.5f;
    v = (local.z / (2*half_d)) + 0.5f;
    if (local.y < 0)
      v = 1 - v; // flip for -y
  } else { // +-Z faces -> use (x,y)
    u = (local.x / (2*half_w)) + 0.5f;
    v = (local.y / (2*half_h)) + 0.5f;
    if (local.z < 0)
      u = 1 - u; // flip for -z
  }
  // repeat pattern - multiply and keep the fractional part of u, v
  u = std::fmod(u*repeatx, 1.0f);
  v = std::fmod(v*repeaty, 1.0f);
  if (u < 0)
    u += 1;
  if (v < 0)
    v += 1;
  unsigned tx = std::min(texture->width - 1,
                         static_cast<unsigned>(u * texture->width));
  unsigned ty = std::min(texture->height - 1,
                         static_cast<unsigned>(v * texture->height));
  return texture->at(ty, tx);
}
