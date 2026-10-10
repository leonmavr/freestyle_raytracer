#ifndef OBJECTS_HPP_
#define OBJECTS_HPP_

#include "vec.hpp"
#include "mat3x3.hpp"
#include "ray.hpp"
#include "common.hpp"
#include <limits> // std::numeric_limits
#include <array>
#include <memory>
#include <cmath>
#include <algorithm>  // clamp


// simple hit record in world coordinates between a ray and an object
struct HitRecord {
  Vec3f where{};
  bool is_hit{false};
  // distance from ray origin to intersection point
  float t{std::numeric_limits<float>::infinity()};
};

struct Sphere; // forward declaration for the downcast helper below

//---------------------------------------------------------------------
// Base classes
//---------------------------------------------------------------------
struct Material {
  Vec3u8 color{50, 235, 220};
  float specular{20};    // 10 -> matte, 100 -> shiny
  float reflective{0};   // 0..1, 0 -> non reflective, 1 -> mirror
  float transparency{0}; // how much light transmitted through it 0..1
  // how much the material refracts light - 1 not at all, > 1 more
  float refractive_index{1};
  float tint{0.1f};      // color tint for refraction (0..1)
  // emissive (glow): added directly to the returned color
  Vec3u8 emission{0, 0, 0};
  float emit_strength{0.0f};
};

struct Object {
  virtual ~Object() = default;
  virtual Vec3f NormalAt(const Vec3f &at) const = 0;
  // only to be overriden for closed solids
  virtual bool IsInside(const Vec3f &point) const {
      return false;
  }
  virtual HitRecord Intersects(const Ray& ray) const = 0;
  // cheap downcast used in hot loops instead of dynamic_cast;
  // nullptr unless overridden by a type that is a Sphere
  virtual const Sphere* AsSphere() const { return nullptr; }
  // base color at point `at` on the surface - override for textures
  virtual Vec3u8 SampleColor(const Vec3f& /*at*/) const {
      return material.color;
  }
  Vec3f center;
  Material material;
};

//---------------------------------------------------------------------
// Objects to render
//---------------------------------------------------------------------
struct Triangle : Object {
  Vec3f v0, v1, v2;

  Triangle() = default;
  Triangle(const Vec3f& a, const Vec3f& b, const Vec3f& c,
           const Material& mat = {});

  virtual Vec3f NormalAt(const Vec3f& /*at*/) const override;

  HitRecord Intersects(const Ray& ray) const;
};

struct Quad : Object{
  Vec3f v0, v1, v2, v3;
  Quad() = default;
  Quad(const Vec3f& a, const Vec3f& b, const Vec3f& c, const Vec3f& d,
       const Material& mat = {});
  virtual Vec3f NormalAt(const Vec3f& /*at*/) const override;
  HitRecord Intersects(const Ray& ray) const;
};

struct Sphere : Object {
  float radius;
  
  Sphere(const Vec3f& c, float r, const Material& mat = {});
  // assuming it's on the sphere
  virtual Vec3f NormalAt(const Vec3f &at) const override;
  virtual bool IsInside(const Vec3f &point) const override;
  virtual const Sphere* AsSphere() const override { return this; }
  // optional texture for sphere using equirectangular mapping
  std::shared_ptr<Image> texture{nullptr};
  int repeat_u{1};
  int repeat_v{1};
  void SetTexture(const std::shared_ptr<Image>& tex,
                  int repeat_u = 1,
                  int repeat_v = 1);
  virtual Vec3u8 SampleColor(const Vec3f& at) const override;
  virtual HitRecord Intersects(const Ray& ray) const override;
};

struct Block : Object {
  Block(Vec3f center, float half_x, float half_y, float half_z,
        const Mat3x3& mrot = {});
  Block(Vec3f center, float half_x, float half_y, float half_z,
        const Mat3x3& mrot,
        const Material& mat);

  virtual Vec3f NormalAt(const Vec3f& at) const override;
  virtual bool IsInside(const Vec3f& point) const override;
  virtual HitRecord Intersects(const Ray& ray) const override;

  // texture mapping 
  std::shared_ptr<Image> texture{nullptr};
  int repeatx{1}; // how many times to repeat texture per face in x axis
  int repeaty{1}; // how many times to repeat texture per face in y axis
  void SetTexture(const std::shared_ptr<Image>& tex,
                  int repeatx = 1,
                  int repeaty = 1);
  virtual Vec3u8 SampleColor(const Vec3f& at) const override;

  std::array<Vec3f, 8> vertices;
  Vec3f axisx, axisy, axisz;
  float half_w, half_h, half_d;
  Mat3x3 rot;
};

#endif // OBJECTS_HPP_
