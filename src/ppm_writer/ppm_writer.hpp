#ifndef PPM_WRITER
#define PPM_WRITER

#include "common.hpp"
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <cstdint>
#include <sstream>

namespace Ppm {

// append an unsigned integer in decimal without ostream overhead
static void AppendUInt(std::string &dst, unsigned v) {
  char tmp[10];
  int n = 0;
  do {
    tmp[n++] = static_cast<char>('0' + v % 10u);
    v /= 10u;
  } while (v);
  while (n) dst.push_back(tmp[--n]);
}

static void Write(const Image &mat,
                   const std::string &filename = "output.ppm") {
  std::ofstream file(filename);
  if (!file)
    throw std::runtime_error("ERROR: Could not write to file " + filename);

  // build the whole file in memory and write it once; formatting
  // millions of channels through operator<< is comparatively slow
  std::string buf;
  buf.reserve(static_cast<std::size_t>(mat.width) * mat.height * 12 + 32);
  buf += "P3\n";
  AppendUInt(buf, mat.width);
  buf += ' ';
  AppendUInt(buf, mat.height);
  buf += "\n255\n";
  for (unsigned y = 0; y < mat.height; ++y) {
    for (unsigned x = 0; x < mat.width; ++x) {
      auto pixel = mat.at(y, x);
      AppendUInt(buf, pixel.x);
      buf += ' ';
      AppendUInt(buf, pixel.y);
      buf += ' ';
      AppendUInt(buf, pixel.z);
      buf += ' ';
    }
    buf += '\n';
  }
  file.write(buf.data(), static_cast<std::streamsize>(buf.size()));
  std::cout << "=== Image saved as " + filename + " ===" << std::endl;
}

/*
 * Handles P3 PPM file format reading and writing:
 * P3
 * <width> <height>
 * <max value>
 * <uint8 uint8 uint8> <uint8 uint8 uint8> ...
 * <uint8 uint8 uint8> <uint8 uint8 uint8> ...
*/
static void Read(Image& dest, const std::string &filename) {
  std::ifstream file(filename);
  if (!file)
    throw std::runtime_error("ERROR: Could not open file " + filename);

  std::string line;
  do {
    std::getline(file, line);
  } while (line.empty() || line[0] == '#');
  if (line != "P3")
    throw std::runtime_error("ERROR: "+ filename +
                             " is not a P3 PPM file!");

  unsigned width = 0, height = 0, maxval = 0;
  while (true) {
    std::getline(file, line);
    if (line.empty() || line[0] == '#') continue;
    std::istringstream iss(line);
    if (iss >> width >> height) break;
  }

  while (true) {
    std::getline(file, line);
    if (line.empty() || line[0] == '#') continue;
    std::istringstream iss(line);
    if (iss >> maxval) break;
  }

  dest = Image(width, height);
  // read pixel values as (uint8, uint8, uint8)
  unsigned x = 0, y = 0;
  while (file && y < height) {
    std::getline(file, line);
    if (line.empty() || line[0] == '#') continue;
    std::istringstream iss(line);
    int r, g, b;
    while (iss >> r >> g >> b) {
      dest.at(y, x++) = Vec3u8(static_cast<uint8_t>(r),
                               static_cast<uint8_t>(g),
                               static_cast<uint8_t>(b));
      if (x == width) {
        x = 0;
        ++y;
      }
    }
  }
}


} // namespace Ppm

#endif // PPM_WRITER
