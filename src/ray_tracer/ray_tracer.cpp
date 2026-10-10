#include "ray_tracer.hpp"
#include <algorithm>


static inline Vec3u8 AddScaled(Vec3u8 base, const Vec3u8& c, float s) {
  if (s <= eps) return base;
  return Vec3u8{
    static_cast<uint8_t>(std::min(255.0f,
                         static_cast<float>(base.x+ c.x * s))),
    static_cast<uint8_t>(std::min(255.0f,
                         static_cast<float>(base.y + c.y * s))),
    static_cast<uint8_t>(std::min(255.0f,
                         static_cast<float>(base.z + c.z * s))),
  };
}

static inline Vec3u8 EmissiveColor(Vec3u8 base, const Material& m) {
  // base + emission * strength
  return AddScaled(base, m.emission, m.emit_strength);
}

RayTracer::RayTracer(const Camera& camera, Lights& lights) :
  camera_(camera),
  image_(camera.width(), camera.height()),
  lights_(lights),
  has_background_(false) {}

const Image& RayTracer::image() { return image_; }

void RayTracer::Trace(int max_reflections) {
  lights_.Normalize();
  // current camera plane corners (world-space)
  auto corners = camera_.CornersWorld();
  // local camera axes in world space for rasterization
  Vec3f world_tl = corners[0];
  Vec3f world_tr = corners[1];
  Vec3f world_bl = corners[2];
  // horizontal (u) and vertical (v) world span vectors,
  // centerd at origin
  Vec3f span_h = world_tr - world_tl;
  Vec3f span_v = world_bl - world_tl;
  // TODO: cam_forward_, cam_right_, cam_up_ are only needed for
  // background sampling -> move them there
  // unit forward, right, and up unprojection span vectors (world)
  cam_forward_ = (camera_.Unproject(0, 0) - camera_.center()).Unit();
  cam_right_ = (world_tr - world_tl).Unit();
  cam_up_ = (world_bl - world_tl).Unit();
  std::size_t w = camera_.width();
  std::size_t h = camera_.height();
  // row-major order keeps writes to the image buffer sequential
  for (std::size_t row{}; row < h; ++row) {
    float v = static_cast<float>(row) / static_cast<float>(h - 1);
    for (std::size_t col{}; col < w; ++col) {
      // normalized column coordinate
      float u = static_cast<float>(col) / static_cast<float>(w - 1);
      // bilinear point on the (possibly rotated) image plane
      Vec3f point_world = world_tl + span_h * u + span_v * v;
      Ray ray(camera_.center(), point_world);
      // Always write the color; TraceRay fills background on miss
      image_.at(row, col) = TraceRay(ray, max_reflections).color;
    }
  }
}

// gamma < 1 brightens the low intensitites of the image,
// gamma > 1 darkens the high intensities
void RayTracer::GammaCorrect(float gamma) {
  // precompute the intensity gamma remap
  uint8_t lut[256];
  for (int i = 0; i < 256; ++i) {
    lut[i] = static_cast<uint8_t>(
        std::pow(static_cast<float>(i) / 255.0f, gamma) * 255.0f);
  }
  for (std::size_t row{}; row < image_.height; ++row) {
    for (std::size_t col{}; col < image_.width; ++col) {
      Vec3u8& px = image_.at(row, col);
      px.x = lut[px.x];
      px.y = lut[px.y];
      px.z = lut[px.z];
    }
  }
}

void RayTracer::SetBackground(const std::string& filename) {
  Ppm::Read(background_, filename);
  has_background_ = (background_.width > 0 && background_.height > 0);
}

float RayTracer::SurroundingIOR(const Vec3f& where,
                                const Object* self,
                                const Vec3f& outward_normal) const {
  Vec3f probe = where + outward_normal * eps * eps_factor;
  float ret = 1.0f; // default is air
  for (const auto& other : objects_) {
    if (other.get() == self) continue;
    if (other->IsInside(probe)) {
      ret = other->material.refractive_index;
    }
  }
  return ret;
}

RayTracer::OrientationInfo RayTracer::RayOrientation(const TraceRecord& record,
                                                     const Vec3f& I,
                                                     float ior_current) const {
  OrientationInfo ret;
  ret.entering = record.normal.Dot(I) < 0;
  // normal; pointing away from  the refracted medium
  ret.normal = ret.entering ? record.normal : -record.normal;
  float n_obj = record.obj->material.refractive_index;
  float n_surrounding = SurroundingIOR(record.hit_point,
                                       record.obj,
                                       record.normal);
  ret.n1 = !ret.entering ? n_surrounding : ior_current;
  ret.n2 = !ret.entering ? n_obj : ior_current;
  ret.eta = ret.n1 / ret.n2;
  ret.cos_i = -ret.normal.Dot(I);
  return ret;
}

std::pair<bool, Vec3f> RayTracer::Refract(const Vec3f &incident,
                                          const Vec3f &N,
                                          float eta,
                                          float cosi) const {
  //cosi = -N.Dot(incident);
  float k = 1.0f - eta * eta * (1.0f - cosi * cosi);
  if (k < 0.0f) return {true, {}}; // TIR
  float cost = std::sqrt(std::max(0.0f, k));
  // formula for trasnmitted (refracted) ray
  Vec3f trans = (incident * eta + N * (eta * cosi - cost)).Unit();
  return {false, trans};
}

float RayTracer::Schlick(float n1, float n2, float cos_i) const {
  float r0 = (n1 - n2) / (n1 + n2);
  return r0*r0 + (1.0f - r0*r0) * std::pow(1.0f - cos_i, 5.0f);
}

TraceRecord RayTracer::TraceRay(const Ray& ray, int depth, float ior_current) const {
  TraceRecord ret;
  // find nearest intersection
  for (const auto& obj : objects_) {
    auto hit = obj->Intersects(ray);
    // reject hits that are behind the ray's origin
    if (!hit.is_hit || hit.t <= 0) continue;

    if (hit.t < ret.t) {
      ret.t = hit.t;
      ret.hit = true;
      ret.hit_point = hit.where;
      ret.obj = obj.get();
      ret.normal = obj->NormalAt(hit.where);
    }
  }

  if (!ret.hit) {
    // no hit; return background color contribution
    ret.color = (has_background_) ? SampleBackground(ray.dir)
                                  : background_color_;
    // add a glow (halo) for rays passing near emitters
    ret.color = AddScaled(ret.color, EmissiveGlow(ray), 1.0f);
    return ret;
  }

  // material parameters
  float mat_trans = std::clamp(ret.obj->material.transparency,
                               0.0f, 1.0f);
  float mat_refl = std::clamp(ret.obj->material.reflective,
                              0.0f, 1.0f);
  // direct lighting (surface shading) from diffuse and specular light
  Vec3u8 direct = lights_.ColorAt(objects_, *ret.obj, ret.hit_point,
                                  camera_);
  // transparency reduces the brightness
  direct *= (1.0f - mat_trans);

  // no need for more ray bounces (base case), only direct ligting 
  if (depth <= 0 || (mat_refl < eps && mat_trans < eps)) {
    ret.color = EmissiveColor(direct, ret.obj->material);
    return ret;
  }

  // incident (pointing away from origin, against the normal)
  const Vec3f& inc = ray.dir; 
  auto ori = RayOrientation(ret, inc, ior_current);
  // cos_i refers to the angle between normal and incident
  float n1 = ori.n1, n2 = ori.n2, eta = ori.eta, cos_i = ori.cos_i;

  //------------------------------------------------------------------
  // reflection
  //------------------------------------------------------------------
  Ray ray_refl{{}, {}};
  ray_refl.dir = inc.ReflectAbout(ori.normal);
  // offset slightly along the normal to avoid self-intersection
  ray_refl.origin = ret.hit_point + ori.normal * eps * eps_factor;
  // -----> child ray (1): reflection
  Vec3u8 refl_col = TraceRay(ray_refl, depth - 1, n1).color;

  //------------------------------------------------------------------
  // refraction 
  //------------------------------------------------------------------
  // Fresnel blending via Schlik approximation for the weights
  float R_fresnel = Schlick(n1, n2, cos_i);
  // (1 - R) * trans = fraction that got trasnmitted
  float trans_weight = mat_trans * (1.0f - R_fresnel);
  // R * trans = fraction that got reflected instead of refracted
  float refl_weight = mat_refl + R_fresnel * mat_trans;
  bool tir = false;
  Vec3u8 refr_color{0, 0,0};
  if (mat_trans > eps) {
    auto [is_tir, refr_dir] = Refract(inc, ori.normal, eta, cos_i);
    tir = is_tir;
    if (!tir) {
      Ray refr_ray{{}, {}};
      refr_ray.origin = ret.hit_point + refr_dir * eps * eps_factor;
      refr_ray.dir = refr_dir;
      // -----> child ray (2): refraction
      refr_color = TraceRay(refr_ray, depth - 1, n2).color;
      // tint heuristic: w = (1 - tint_w) + tint_w * (color / 255)
      float tint_w = ret.obj->material.tint * mat_trans;
      auto ApplyTint = [&](uint8_t col_next,
                           uint8_t color_curr)->uint8_t{
        float curr_norm = static_cast<float>(color_curr) / 255.0f;
        float w = (1.0f - tint_w) + tint_w * curr_norm;
        return static_cast<uint8_t>(std::min(255.0f, col_next * w));
      };
      auto color_current = ret.obj->material.color;
      refr_color = Vec3u8{
        ApplyTint(refr_color.x, color_current.x),
        ApplyTint(refr_color.y, color_current.y),
        ApplyTint(refr_color.z, color_current.z)
      };
    } else {
      // total internal reflection:
      // send transparency energy into reflection
      trans_weight = 0.0f;
      refl_weight = std::min(1.0f, refl_weight + mat_trans);
    }
  }

  float total = refl_weight + trans_weight;
  float w_direct = 1.0f - std::min(total, 1.0f);
  ret.color = Vec3u8{
    static_cast<uint8_t>(direct.x * w_direct +
                         refl_col.x * refl_weight +
                         refr_color.x * trans_weight),
    static_cast<uint8_t>(direct.y * w_direct +
                         refl_col.y * refl_weight +
                         refr_color.y * trans_weight),
    static_cast<uint8_t>(direct.z * w_direct +
                         refl_col.z * refl_weight +
                         refr_color.z * trans_weight)
  };
  // add glow accumulated in air if no reflection or refraction
  const Vec3u8& emissive = EmissiveGlow(ray, std::max(0.0f, ret.t));
  ret.color = AddScaled(ret.color, emissive, 1.0f);
  return ret;
}

Vec3u8 RayTracer::EmissiveGlow(const Ray& ray, float t_max) const {
  // Add a soft halo for rays that pass near emissive emitters.
  // Obviously not physically correct - just a cheap approximation.
  // As the ray travels close to an emissive source, it picks up its
  // glow (halo) from the closest point.

  // glow accumulator from all emissive sources
  Vec3f color_accum{0.0f, 0.0f, 0.0f};
  const Vec3f& D = ray.dir;
  const float t_end = std::max(0.0f, t_max);
  // add all intensitiies from non-intersecting emissive sources
  for (const auto& obj : objects_) {
    // nullptr unless the object is a Sphere (avoids RTTI downcast)
    const Sphere* s = obj->AsSphere();
    if (s) {
      const Material& m = s->material;
      if (m.emit_strength <= eps) continue;
      // closest point on the ray segment [0, t_end]
      float t = (s->center - ray.origin).Dot(D);
      t = std::clamp(t, 0.0f, t_end);
      if (t <= 0.0f) continue;

      const Vec3f closest = ray.At(t);
      const float d_closest = (closest - s->center).Norm();
      // direct hit (handled by intersection logic)
      if (d_closest < s->radius) continue; 
      constexpr float falloff_strength = 0.3f;
      const float falloff = s->radius * falloff_strength;
      // Light attenuation based on Beer-Lambert law:
      // sciencedirect.com/topics/engineering/beer-lambert-law
      // exponential falloff away from the emitter silhouette
      // lower bound to zero in case it's inside the sphere
      const float w = std::exp(-std::max(d_closest - s->radius, 0.0f)
                               /falloff);
      const float halo_strength = 0.45f * m.emit_strength * w;
      color_accum += m.emission * halo_strength;
      continue;
    }
  }
  return Vec3u8{
    static_cast<uint8_t>(std::min(255.0f, color_accum.x)),
    static_cast<uint8_t>(std::min(255.0f, color_accum.y)),
    static_cast<uint8_t>(std::min(255.0f, color_accum.z)),
  };
}

// Compute a far sample world point along the ray from the current
// camera center,
// -> map that point's two spherical angles with wrapping
// -> map (scale) those those to background pixel coordinates
Vec3u8 RayTracer::SampleBackground(const Vec3f& dir_world) const {
  if (!has_background_) return background_color_;
  // heuristic: choose a point n*camera_depth times away in the
  // world's direction to set it as parallax; the larger the n,
  // the less sensitive the background to translation
  const float z_parallax = 15 * camera_.depth();
  Vec3f sample_world = camera_.center() + dir_world * z_parallax;
  // Convert the world point to spherical angles relative to world origin.
  float x = sample_world.x;
  float y = sample_world.y;
  float z = sample_world.z;
  // yaw in [-pi, pi], pitch in [-pi/2, pi/2]
  float yaw = std::atan2(x, z);
  float pitch = std::atan2(y, std::sqrt(x*x + z*z));
  // map to [0,1) x [0,1] with horizontal wrap
  float u = (yaw + static_cast<float>(M_PI)) / (2.0f * static_cast<float>(M_PI));
  u = std::fmod(u, 1.0f);
  if (u < 0.0f) u += 1.0f;
  float v = (pitch + static_cast<float>(M_PI) * 0.5f) / static_cast<float>(M_PI);
  v = std::clamp(v, 0.0f, 1.0f);
  // convert to pixel indices (nearest neighbor)
  unsigned w = background_.width;
  unsigned h = background_.height;
  unsigned px = std::min(static_cast<unsigned>(u * w), w ? w - 1 : 0);
  unsigned py = std::min(static_cast<unsigned>(v * h), h ? h - 1 : 0);
  return background_.at(py, px);
}
