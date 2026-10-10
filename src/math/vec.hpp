#ifndef VEC_HPP_
#define VEC_HPP_

#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>

// forward declaration for aliases
template <typename T> struct Xyz;
// aliases for different types of data - use those for operations
using Vec3u8 = Xyz<uint8_t>;
using Vec3i32 = Xyz<int32_t>;
using Vec3f = Xyz<float>;

template <typename T> struct Xyz {
  // array xyz and x,y,z overlay in memory so x,y,z alias xyz[0],[1],[2]
  union {
    struct {
      T x, y, z;
    };
    T xyz[3];
  };
  Xyz() : x(T{}), y(T{}), z(T{}) {}
  Xyz(T xx, T yy, T zz) : x(xx), y(yy), z(zz) {}
  // explicit to avoid converting floats or ints to Xyz
  explicit Xyz(T val) : x(val), y(val), z(val) {}
  // all members trivially copyable - no need to define copy/move c/tors
  Xyz(const Xyz &) = default;
  Xyz(Xyz &&) noexcept = default;
  Xyz &operator=(const Xyz &) = default;
  Xyz &operator=(Xyz &&) noexcept = default;
  //--------------------------------------------------------------------
  // vector to vector overloaded operators
  //--------------------------------------------------------------------
#define VECTOR_OPS(OP)                                                         \
  template <typename U>                                                        \
  auto operator OP(const Xyz<U> &other) const->Xyz<decltype(T {} OP U{})> {    \
    return Xyz<decltype(T {} OP U{})>(x OP other.x, y OP other.y,              \
                                      z OP other.z);                           \
  }                                                                            \
                                                                               \
  template <typename U> Xyz<T> &operator OP##=(const Xyz<U> &other) {          \
    x OP## = other.x;                                                          \
    y OP## = other.y;                                                          \
    z OP## = other.z;                                                          \
    return *this;                                                              \
  }

  VECTOR_OPS(+)
  VECTOR_OPS(-)
  VECTOR_OPS(*)
#undef VECTOR_OPS
  //--------------------------------------------------------------------
  // vector to scalar overloaded operator
  //--------------------------------------------------------------------
  // int type OP floating type operations are not properly supported
#define SCALAR_OPS(OP)                                                         \
  template <typename U>                                                        \
  auto operator OP(const U scalar) const->Xyz<decltype(T {} OP U{})> {         \
    return Xyz<decltype(T {} OP U{})>(xyz[0] OP scalar, xyz[1] OP scalar,      \
                                      xyz[2] OP scalar);                       \
  }                                                                            \
  template <typename U> Xyz<T> &operator OP##=(const U scalar) {               \
    x OP## = scalar;                                                           \
    y OP## = scalar;                                                           \
    z OP## = scalar;                                                           \
    return *this;                                                              \
  }
  SCALAR_OPS(+)
  SCALAR_OPS(-)
  SCALAR_OPS(*)
  SCALAR_OPS(/)
#undef SCALAR_OPS
  //--------------------------------------------------------------------
  // equality operators
  //--------------------------------------------------------------------
  bool operator==(const Xyz<T> &other) const {
    if constexpr (std::is_floating_point_v<T>) {
      constexpr float eps = static_cast<float>(1e-3);
      return (std::fabs(x - other.x) < eps) && (std::fabs(y - other.y) < eps) &&
             (std::fabs(z - other.z) < eps);
    } else {
      return (x == other.x) && (y == other.y) && (z == other.z);
    }
  }

  bool operator!=(const Xyz<T> &other) const { return !(*this == other); }
  //--------------------------------------------------------------------
  // other operators
  //--------------------------------------------------------------------
  // unary negate
  Xyz<T> operator-() const { return Xyz<T>(-xyz[0], -xyz[1], -xyz[2]); }

  // index - useful for matrix operations later
  T operator[](int i) const {
    if (i < 0 || i >= 3) [[unlikely]]
      throw std::out_of_range("Vector index out of range");
    return xyz[i];
  }
  T &operator[](int i) {
    if (i < 0 || i >= 3) [[unlikely]]
      throw std::out_of_range("Vector index out of range");
    return xyz[i];
  }

  // when doing std::cout << xyz;
  friend std::ostream &operator<<(std::ostream &os, const Xyz &xyz) {
    os << "[" << xyz[0] << ", " << xyz[1] << ", " << xyz[2] << "]";
    return os;
  }
  //--------------------------------------------------------------------
  // common vector operations
  //--------------------------------------------------------------------
  T Dot(const Xyz<T> &other) const {
    return x * other.x + y * other.y + z * other.z;
  }

  float NormSq() const { return x * x + y * y + z * z; }

  float Norm() const { return std::sqrt(NormSq()); }

  Xyz<float> Unit() const {
    const float n = Norm();
    return Xyz<float>{x / n, y / n, z / n};
  }

  float Cos(const Xyz<T> &other) const {
    if (this != &other)
      return Dot(other) / (Norm() * other.Norm());
    return 1.0;
  }

  float Angle(const Xyz<T> &other) const { return std::acos(Cos(other)); }
  Xyz<T> Cross(const Xyz<T> &other) const {
    return Xyz<T>(y * other.z - z * other.y, z * other.x - x * other.z,
                  x * other.y - y * other.x);
  }

  Xyz<T> ReflectAbout(const Xyz<T> &axis) const {
    // reflect the vector about the given axis (treated as normal)
    const auto n = axis.Unit();
    double v = Dot(n);
    return Xyz<T>{
      static_cast<T>(x - 2 * v * n.x),
      static_cast<T>(y - 2 * v * n.y),
      static_cast<T>(z - 2 * v * n.z)};
  }
};

#endif // VEC_HPP_
