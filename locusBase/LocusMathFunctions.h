#pragma once
#include "Core.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <math.h>
#include <raylib.h>
#include <sys/types.h>
#include <tuple>
#include <vector>

namespace locusMath {
class LMathFunctions {
public:
  static locus::real Magnitude(const locus::Vector3 &v) {
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
  }

  static locus::real SquareMagnitude(const locus::Vector3 &v) {
    return (v.x * v.x + v.y * v.y + v.z * v.z);
  }

  static locus::real Magnitude(const locus::Point &v) {
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
  }
  static locus::Vector3 Normalize(const locus::Vector3 &v) {
    locus::real mag = Magnitude(v);
    locus::Vector3 normalized = locus::Vector3(v.x / mag, v.y / mag, v.z / mag);
    return normalized;
  }

  static locus::Vector3 CrossProduct(const locus::Vector3 &a,
                                     const locus::Vector3 &b) {
    locus::real x = a.y * b.z - a.z * b.y;
    locus::real y = a.z * b.x - a.x * b.z;
    locus::real z = a.x * b.y - a.y * b.x;

    return locus::Vector3(x, y, z);
  }

  static locus::Point CrossProduct(const locus::Point &a,
                                   const locus::Point &b) {
    locus::real x = a.y * b.z - a.z * b.y;
    locus::real y = a.z * b.x - a.x * b.z;
    locus::real z = a.x * b.y - a.y * b.x;

    return locus::Point{.x = x, .y = y, .z = z};
  }

  static locus::real CrossProduct2D(locus::real x1, locus::real y1,
                                    locus::real x2, locus::real y2) {
    return (x1 * y2) - (x2 * y1);
  }

  static locus::real DotProduct(const locus::Point &a, const locus::Point &b) {
    locus::real x = a.x * b.x;
    locus::real y = a.y * b.y;
    locus::real z = a.z * b.z;
    return x + y + z;
  }

  static locus::Point Normalize(const locus::Point &a) {
    locus::real magnitude = Magnitude(a);
    locus::real x = a.x / magnitude;
    locus::real y = a.y / magnitude;
    locus::real z = a.z / magnitude;

    return locus::Point{.x = x, .y = y, .z = z};
  }

  // computes barycentric cordinates for triangle
  // From u=PBC/ABC , v=PCA/ABC
  static std::tuple<locus::real, locus::real, locus::real>
  BaryCentric(const locus::Point &p, const locus::Point &a,
              const locus::Point &b, const locus::Point &c) {

    // uA + vB + wC = P
    // u = PBC/ABC
    // v = PCA/ABC
    locus::real u = 0.f;
    locus::real v = 0.f;
    locus::real w = 0.f;
    // area of actual triangle ABC
    locus::Point AB{.x = b.x - a.x, .y = b.y - a.y, .z = b.z - a.z};
    locus::Point AC{.x = c.x - a.x, .y = c.y - a.y, .z = c.z - a.z};
    locus::Point ABC = CrossProduct(AB, AC);
    // determine best view plane
    locus::real x = std::abs(ABC.x), y = std::abs(ABC.y), z = std::abs(ABC.z);
    locus::real PBC = 0.f;
    locus::real PCA = 0.f;
    locus::real denom = 0.f;
    // yz plane
    if (x >= y && x >= z) {
      // PBC
      locus::Vector3 PB(0.f, (p.y - b.y), (p.z - b.z));
      locus::Vector3 BC(0.f, (b.y - c.y), (b.z - c.z));
      PBC = CrossProduct2D(PB.y, PB.z, BC.y, BC.z);
      // PCA
      locus::Vector3 PC(0.f, (p.y - c.y), (p.z - c.z));
      locus::Vector3 CA(0.f, (c.y - a.y), (c.z - a.z));
      PCA = CrossProduct2D(PC.y, PC.z, CA.y, CA.z);
      denom = ABC.x;
    }
    // xz plane
    else if (y >= x && y >= z) {
      // PBC
      locus::Vector3 PB((p.x - b.x), 0, (p.z - b.z));
      locus::Vector3 BC((b.x - c.x), 0, (b.z - c.z));
      PBC = CrossProduct2D(PB.x, PB.z, BC.x, BC.z);
      // PCA
      locus::Vector3 PC((p.x - c.x), 0, (p.z - c.z));
      locus::Vector3 CA((c.x - a.x), 0, (c.z - a.z));
      PCA = CrossProduct2D(PC.x, PC.z, CA.x, CA.z);
      denom = -ABC.y;
    }
    // xy planes
    else {
      // PBC
      locus::Vector3 PB((p.x - b.x), (p.y - b.y), 0);
      locus::Vector3 BC((b.x - c.x), (b.y - c.y), 0);
      PBC = CrossProduct2D(PB.x, PB.y, BC.x, BC.y);
      // PCA
      locus::Vector3 PC((p.x - c.x), (p.y - c.y), 0);
      locus::Vector3 CA((c.x - a.x), (c.y - a.y), 0);
      PCA = CrossProduct2D(PC.x, PC.y, CA.x, CA.y);
      denom = ABC.z;
    }
    u = PBC / denom;
    v = PCA / denom;
    w = (1 - u - v);
    return {u, v, w};
  }

  static bool TestPointInTriangle(const locus::Point &p, const locus::Point &a,
                                  const locus::Point &b,
                                  const locus::Point &c) {
    auto [u, v, w] = BaryCentric(p, a, b, c);
    return u >= 0 && v >= 0 && (u + v) <= 1;
  }

  // Computes plane equation given three points forming a triangle in ccw
  static locus::Plane ComputePlane(const locus::Point &a, const locus::Point &b,
                                   const locus::Point &c) {
    locus::Plane p;
    locus::Point _n = Normalize(CrossProduct(b - a, c - a));
    locus::Vector3 n(_n.x, _n.y, _n.z);
    p.n = n;
    p.d = locusMath::LMathFunctions::DotProduct(_n, a);
    return p;
  }
  // Checks if Quad is convex
  // returns true if convex
  static bool isConvexQuad(const locus::Point &a, const locus::Point &b,
                           const locus::Point &c, const locus::Point &d) {
    // dot(normal(ABC), normal(ACD) )
    locus::real dot = DotProduct(CrossProduct((d - b), (a - b)),
                                 CrossProduct((d - b), (c - b)));
    if (dot >= 0.0f) {
      return false;
    }
    return true;
  }

  static bool ispointIsRightOfEdge2D(const locus::Point &A,
                                     const locus::Point B,
                                     const locus::Point &p) {
    locus::Point AB = B - A;
    locus::Point AP = p - A;
    locus::real cross = CrossProduct2D(AB.x, AB.y, AP.x, AP.y);
    return cross > 0.0f;
  }
  // Constructs convex hull on x y coordinate
  //
  static std::vector<locus::Point>
  AndrewConvexHullXY(std::vector<locus::Point> points, bool upper = true) {
    // sort points
    std::sort(points.begin(), points.end(),
              [](const locus::Point &a, const locus::Point &b) {
                return a.x <= b.x;
              });

    std::vector<locus::Point> convexHullPoints = {points[0], points[1]};
    locus::Point currentEdge;
    locus::Point A;
    locus::Point B;
    locus::Point p;
    for (size_t i = 2; i < points.size(); i++) {
      A = convexHullPoints[convexHullPoints.size() - 1];
      B = convexHullPoints[convexHullPoints.size() - 2];
      p = points[i];
      if (upper) {
        if (ispointIsRightOfEdge2D(A, B, p) || convexHullPoints.size() < 2) {
          convexHullPoints.push_back(p);
          continue;
        }
      } else {
        if (!ispointIsRightOfEdge2D(A, B, p) || convexHullPoints.size() < 2) {
          convexHullPoints.push_back(p);
          continue;
        }
      }

      convexHullPoints.pop_back();
      i--; // check the current point again on next iteration for the current
           // last edge after removal above
    }
    return convexHullPoints;
  }

  // Finds point farthest from edge ab
  static int32_t pointFarthestFromEdge(const locus::Point &a,
                                       const locus::Point &b,
                                       std::vector<locus::Point> &setOfPoints,
                                       bool upper = true) {
    locus::Point edge = (b - a), edgePerp = {-edge.y, edge.x, 0.f};
    if (upper) {
      edgePerp = {edge.y, -edge.x, 0.f};
    }
    int32_t bestIndex = -1;
    float dMax = -INFINITY;
    float rMax = -INFINITY;
    for (size_t i = 0; i < setOfPoints.size(); i++) {
      locus::Point p = setOfPoints[i];
      locus::real d = DotProduct(edgePerp, p);
      locus::real r = DotProduct(edge, p - a);
      if (d > dMax || (d == dMax && r > rMax)) {
        bestIndex = i;
        dMax = d;
        rMax = r;
      }
    }
    return bestIndex;
  }
};
} // namespace locusMath
