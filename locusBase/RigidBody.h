#include "Core.h"

namespace locusBody {
using locus::Matrix4;
using locus::Quaternion;
using locus::real;
using locus::Vector3;
using locus::Matrix3;

class RigidBody {
protected:
  real inverseMass;
  Vector3 position;
  Vector3 linearVelocity;
  Vector3 angularVelocity;
  Quaternion orientation;
  Matrix4 transformationMatrix;
  Matrix3 inverseInertiaTensorLocal;
  Matrix3 inverseInertiaTensorWorld;
  static inline void _calculateInverseInetiaTensorWorld(Matrix3 &inverseInertiaTensorWorld, const Matrix4 &transformationMatrix, const Matrix3 &inverseInertiaTensorLocal);
public:
    void calculateDerivedData();
    void setInverseInertiaTensorLocal(const Matrix3 &inertiaTensor);
};
} // namespace locusBody
