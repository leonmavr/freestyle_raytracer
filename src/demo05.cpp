#include "camera.hpp"
#include "light.hpp"
#include "ppm_writer.hpp"
#include "ray_tracer.hpp"
#include "vec.hpp"
#include "material_builder.hpp"
#include "common.hpp"
#include <string>
#include <cmath>
#include <memory>
#include <iostream>

// Refraction and total internal reflection (TIR) test scene
//
// Two glass objects (n = 1.5) inside a room with checkerboard walls of
// different colors, so refracted rays always land on something visible:
// - sphere: acts as a lens, i.e. the far wall appears shrunk and flipped
// - cube rotated by 45 deg: rays entering near its outer edges exceed
//   the critical angle at the back faces, undergo TIR and exit sideways,
//   showing the side walls of the room
//
// wall colors (as seen from the default camera):
//   far wall: green (top-left), red (top-right), yellow (bottom-left),
//             blue (bottom-right)
//   left: cyan, right: purple, floor: orange, ceiling: teal,
//   behind the camera: pink
namespace {

// checkerboard texture of cells_x x cells_y cells; `quadrants` colors
// each quarter of the board differently (used for the far wall)
std::shared_ptr<Image> Checker(int cells_x, int cells_y, Vec3u8 c,
                               bool quadrants = false) {
  constexpr int cell = 32;
  auto img = std::make_shared<Image>(cells_x * cell, cells_y * cell);
  for (unsigned r = 0; r < img->height; ++r) {
    for (unsigned col = 0; col < img->width; ++col) {
      const int cx = col / cell, cy = r / cell;
      Vec3u8 color = c;
      if (quadrants) {
        const bool left = cx < cells_x / 2, top = cy < cells_y / 2;
        color = top ? (left ? Vec3u8{220, 40, 40} : Vec3u8{40, 190, 60})
                    : (left ? Vec3u8{40, 80, 230} : Vec3u8{235, 205, 30});
      }
      img->at(r, col) = ((cx + cy) % 2) ? Vec3u8{245, 245, 245} : color;
    }
  }
  return img;
}

} // namespace

int main(int argc, char** argv) {
  constexpr int depth = 600, fovx_deg = 64, fovy_deg = 44;
  std::string output_file = "output.ppm";
  Vec3f cam_center{0, 0, 0};
  Mat3x3 cam_rot{};
  // first argument is the output file
  if (argc > 1 && argv[1] != nullptr)
    output_file = std::string(argv[1]);
  // next 3 arguments are the camera center
  if (argc >= 5) {
    try {
      cam_center = Vec3f{std::stof(argv[2]), std::stof(argv[3]),
                         std::stof(argv[4])};
    } catch (...) {
      std::cerr << "Invalid camera center, using defaults." << std::endl;
    }
  }
  // next 3 are the camera rotation (degrees)
  if (argc >= 8) {
    try {
      cam_rot = Mat3x3(Deg2Rad(std::stof(argv[5])),
                       Deg2Rad(std::stof(argv[6])),
                       Deg2Rad(std::stof(argv[7])));
    } catch (...) {
      std::cerr << "Invalid camera rotation, using defaults." << std::endl;
    }
  }

  Camera cam(depth, fovx_deg, fovy_deg, cam_center, cam_rot);

  Lights lights;
  lights.AddAmbient(0.8);
  lights.AddPoint(0.4, 0, -800, -600);
  RayTracer ray_tracer(cam, lights);

  // closed room of thin textured blocks (y < 0 is up)
  constexpr float X = 1600, Y = 1000, Z0 = -1500, Z1 = 2600, th = 10;
  constexpr float zc = (Z0 + Z1) / 2, zh = (Z1 - Z0) / 2;
  const Material wall = MaterialBuilder().Specular(0).Build();
  auto add_wall = [&](Vec3f c, float hx, float hy, float hz,
                      std::shared_ptr<Image> tex) {
    auto& b = ray_tracer.AddObject<Block>(c, hx, hy, hz, Mat3x3{}, wall);
    b.SetTexture(tex);
  };
  // far wall and wall behind the camera
  add_wall({0, 0, Z1 + th}, X, Y, th, Checker(16, 10, {}, true));
  add_wall({0, 0, Z0 - th}, X, Y, th, Checker(16, 10, {240, 110, 170}));
  // left and right walls
  add_wall({-X - th, 0, zc}, th, Y, zh, Checker(20, 10, {0, 190, 200}));
  add_wall({X + th, 0, zc}, th, Y, zh, Checker(20, 10, {140, 60, 200}));
  // floor and ceiling
  add_wall({0, Y + th, zc}, X, th, zh, Checker(16, 20, {240, 130, 20}));
  add_wall({0, -Y - th, zc}, X, th, zh, Checker(16, 20, {20, 120, 110}));

  Material glass = MaterialBuilder()
                     .Color(255, 255, 255)
                     .Specular(120)
                     .Reflective(0.0f)
                     .Transparency(0.95f)
                     .RefractiveIndex(1.5f)
                     .Tint(0.0f)
                     .Build();
  ray_tracer.AddObject<Sphere>(Vec3f{-330, 0, 1300}, 260, glass);
  ray_tracer.AddObject<Block>(Vec3f{330, 0, 1300}, 180, 180, 180,
                              Mat3x3{0.0f, static_cast<float>(M_PI) / 4,
                                     0.0f},
                              glass);

  constexpr int nreflections = 6;
  ray_tracer.Trace(nreflections);
  Ppm::Write(ray_tracer.image(), output_file);
}
