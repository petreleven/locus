#pragma once

#include <math.h>
namespace locus {
typedef float real;
class Vector3 {
public:
  real x;
  real y;
  real z;

private:
  real pad;
  real real_sqrt(real squareMag) { return sqrtf(squareMag); }

public:
  Vector3() : x(0), y(0), z(0) {}
  Vector3(const real x, const real y, const real z) : x(x), y(y), z(z) {}

  // ADD TO THIS
  void operator+=(const Vector3 &v) {
    x += v.x;
    y += v.y;
    z += v.z;
  }
  Vector3 operator+(locus::real &sc) {
    x += sc;
    y += sc;
    z += sc;
    return Vector3(x, y, z);
  }

  // SUBSTRACT FROM THIS
  void operator-=(const Vector3 &v) {
    x -= v.x;
    y -= v.y;
    z -= v.z;
  }
  // SUUBSTRACT AND RETURN NEW Vec3
  Vector3 operator-(const Vector3 v) const {
    return Vector3(x - v.x, y - v.y, z - v.z);
  }

  Vector3 &operator=(const Vector3 &v) {
    x = v.x;
    y = v.y;
    z = v.z;
    return *this;
  }
};

struct Point {
  real x;
  real y;
  real z;
  void operator+=(const Vector3 &v) {
    x += v.x;
    y += v.y;
    z += v.z;
  }
  Point operator+(locus::real &sc) {
    x += sc;
    y += sc;
    z += sc;
    return Point{x, y, z};
  }

  // SUBSTRACT FROM THIS
  void operator-=(const Point &v) {
    x -= v.x;
    y -= v.y;
    z -= v.z;
  }
  // SUUBSTRACT AND RETURN NEW Vec3
  Point operator-(const Point v) const {
    return Point{x - v.x, y - v.y, z - v.z};
  }

  Point &operator=(const Point &v) {
    x = v.x;
    y = v.y;
    z = v.z;
    return *this;
  }
};

struct Plane {
  locus::Vector3 n; // plane normal
  locus::real d;    // distance from origin dot(normal, point_on_plane)
};

}; // namespace locus
