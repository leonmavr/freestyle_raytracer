#ifndef LIGHT_HPP_
#define LIGHT_HPP_

#include "ray.hpp"
#include "vec.hpp"
#include "objects.hpp"
#include "camera.hpp"
#include "common.hpp"
#include <vector>
#include <memory>
#include <optional>

enum class LightType : int {
    AMBIENT,
    POINT,
    DIRECTIONAL,
};

struct Light {
    LightType type;
    float intensity;
    // only valid for point/directional light
    std::optional<Vec3f> data{};
};

class Lights {
public:
  void AddAmbient(float intensity);
  void AddPoint(float intensity, float posx, float posy, float posz);
  void AddDir(float intensity, float dirx, float diry, float dirz);
  // call it having added all lights to normalize their intensities
  void Normalize();
  // diffuse and specular light contribution at a point on an object;
  // `normal` is the surface normal already computed at `at`
  Vec3u8 ColorAt(const std::vector<std::unique_ptr<Object>>& objects,
                 const Object &object,
                 const Vec3f &at,
                 const Camera &camera,
                 const Vec3f &normal) const;

private:
  std::vector<Light> lights_;

  static float ShadowFactor(const Light& light,
                            const std::vector<std::unique_ptr<Object>>& objects,
                            const Object& object,
                            const Vec3f& at,
                            const Vec3f& normal);
};

#endif // LIGHT_HPP_
