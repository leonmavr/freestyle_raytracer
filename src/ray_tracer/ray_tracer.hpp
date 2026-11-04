#ifndef RAY_TRACER_HPP_
#define RAY_TRACER_HPP_

#include "common.hpp"
#include "ppm_writer.hpp"
#include "objects.hpp"
#include "light.hpp"
#include "camera.hpp"
#include "ray.hpp"
#include <vector>
#include <memory>
#include <type_traits>
#include <utility>
#include <limits> // numeric_limits
#include <cmath>


struct TraceRecord {
  Vec3u8 color{0, 0, 0};      // final color
  bool hit{false};        
  float t{std::numeric_limits<float>::infinity()}; // hit distance
  Vec3f hit_point{};          // where the hit occured
  Vec3f normal{};             // surface normal at hit
  const Object* obj{nullptr}; // hit object (nullptr if no hit)
};

class RayTracer {
public:
  RayTracer(const Camera& camera, Lights& lights);

  // variadic template to store any object derived from Object
  // pass Args to allow constructing objects in place (c/tor with arguments)
  template <typename T, typename... Args>
  T& AddObject(Args&&... args) {
    static_assert(std::is_base_of_v<Object, T>,
                  "AddObject<T>: T must derive from Object");
    // with std::forward to preserve rvalue/lvalue nature
    auto obj = std::make_unique<T>(std::forward<Args>(args)...);
    T& ref = *obj;
    objects_.push_back(std::move(obj));
    return ref;
  }

  const Image& image();

  void Trace(int max_reflections = 5);

  // gamma < 1 brightens the low intensitites of the image,
  // gamma > 1 darkens the high intensities
  void GammaCorrect(float gamma);

  // background can be a PPM image or a solid color
  void SetBackground(const std::string& filename);
  void SetBackground(uint8_t r, uint8_t g, uint8_t b) {
    has_background_ = false;
    background_color_ = {r, g, b};
  }

private:
  // get the corrent IOR (index of refraction) and normal arrangement
  // for refraction calculations
  struct OrientationInfo {
    bool entering{true};
    // oriented normal, i.e. pointing away from refracted medium
    Vec3f normal{};
    float n1{1.0f};
    float n2{1.0f};
    float eta{1.0f};
    float cos_i{0.0f};
  };

  // probe the refractive index of the surrounding medium slightly off
  // the surface
  float SurroundingIOR(const Vec3f& where,
                       const Object* self,
                       const Vec3f& outward_normal) const;

  // determine normal orientation and the IOR (index of refraction)
  // pair, given that the normal ray should point towadds the incident
  // plane
  OrientationInfo RayOrientation(const TraceRecord& record, 
                                 const Vec3f& I,
                                 float ior_current) const;

  // returns refracted vector (Snell's vectorized law) and whether TIR
  // occured
    std::pair<bool, Vec3f>
    Refract(const Vec3f &incident, /* incident ray direction */
      const Vec3f &N,              /* normal at intersection */
      float eta,                   /* relative refr index n1/n2 */
      float cosi                   /* cos of incident ray */) const;

  // Schlick's approximation of Fresnel reflectance
  float Schlick(float n1, float n2, float cos_i) const;

  TraceRecord TraceRay(const Ray& ray, int depth, float ior_current = 1.0f) const;

  // simple glow halo: add emission for rays that pass near emissive spheres
  Vec3u8 EmissiveGlow(const Ray& ray,
                      float t_max = std::numeric_limits<float>::infinity()) const;

  // map ray direction to background image using equirectangular mapping
  Vec3u8 SampleBackground(const Vec3f& dir_world) const;

  const Camera &camera_;
  std::vector<std::unique_ptr<Object>> objects_;
  // image buffer to store the final colors
  Image image_;
  Lights& lights_;
  // how much to scale the eps of a normal to avoid self-intersection
  static constexpr const float eps_factor = 40.0;
  // cached camera basis for mapping rays to background image
  Vec3f cam_right_{};
  Vec3f cam_up_{};
  Vec3f cam_forward_{};
  // emissive spheres, cached at the start of Trace() for the glow pass
  std::vector<const Sphere*> emissive_{};
  // spherical background
  Image background_{};
  bool has_background_{false};
  // solid background color when no background image is used
  Vec3u8 background_color_{0, 0, 0};
};

#endif // RAY_TRACER_HPP_
