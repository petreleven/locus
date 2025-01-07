#include "RigidBody.h"
#include "Core.h"

namespace locusBody {
//called on the body to set its parameters i.e
// orientaion normalizing
// tx matrix
// Inverse Inertia Tensor in world coordinates
void RigidBody::calculateDerivedData() {
  orientation.normalize();
  transformationMatrix.setOrientation(orientation, position);
  _calculateInverseInetiaTensorWorld(inverseInertiaTensorWorld, transformationMatrix, inverseInertiaTensorLocal);
}


void RigidBody::setInverseInertiaTensorLocal(const Matrix3 &inertiaTensor) {
  inverseInertiaTensorLocal.setInverse(inertiaTensor);
}

// rotMatrix . inverseInerialLocal * rotMxtrixInv;
// think of it like qvq* from quaternion
inline void RigidBody::_calculateInverseInetiaTensorWorld(
    Matrix3 &inverseInertiaTensorWorld, const Matrix4 &transformationMatrix,
    const Matrix3 &inverseInertiaTensorLocal) {

  inverseInertiaTensorWorld.data[0] =
      transformationMatrix.data[0] * inverseInertiaTensorLocal.data[0] +
      transformationMatrix.data[1] * inverseInertiaTensorLocal.data[3] +
      transformationMatrix.data[2] * inverseInertiaTensorLocal.data[6];
  inverseInertiaTensorWorld.data[1] =
      transformationMatrix.data[0] * inverseInertiaTensorLocal.data[1] +
      transformationMatrix.data[1] * inverseInertiaTensorLocal.data[4] +
      transformationMatrix.data[2] * inverseInertiaTensorLocal.data[7];
  inverseInertiaTensorWorld.data[2] =
      transformationMatrix.data[0] * inverseInertiaTensorLocal.data[2] +
      transformationMatrix.data[1] * inverseInertiaTensorLocal.data[5] +
      transformationMatrix.data[2] * inverseInertiaTensorLocal.data[8];

  inverseInertiaTensorWorld.data[3] =
      transformationMatrix.data[4] * inverseInertiaTensorLocal.data[0] +
      transformationMatrix.data[5] * inverseInertiaTensorLocal.data[3] +
      transformationMatrix.data[6] * inverseInertiaTensorLocal.data[6];
  inverseInertiaTensorWorld.data[4] =
      transformationMatrix.data[4] * inverseInertiaTensorLocal.data[1] +
      transformationMatrix.data[5] * inverseInertiaTensorLocal.data[4] +
      transformationMatrix.data[6] * inverseInertiaTensorLocal.data[7];
  inverseInertiaTensorWorld.data[5] =
      transformationMatrix.data[4] * inverseInertiaTensorLocal.data[2] +
      transformationMatrix.data[5] * inverseInertiaTensorLocal.data[5] +
      transformationMatrix.data[6] * inverseInertiaTensorLocal.data[8];

  inverseInertiaTensorWorld.data[6] =
      transformationMatrix.data[8] * inverseInertiaTensorLocal.data[0] +
      transformationMatrix.data[9] * inverseInertiaTensorLocal.data[3] +
      transformationMatrix.data[10] * inverseInertiaTensorLocal.data[6];
  inverseInertiaTensorWorld.data[7] =
      transformationMatrix.data[8] * inverseInertiaTensorLocal.data[1] +
      transformationMatrix.data[9] * inverseInertiaTensorLocal.data[4] +
      transformationMatrix.data[10] * inverseInertiaTensorLocal.data[7];
  inverseInertiaTensorWorld.data[8] =
      transformationMatrix.data[8] * inverseInertiaTensorLocal.data[2] +
      transformationMatrix.data[9] * inverseInertiaTensorLocal.data[5] +
      transformationMatrix.data[10] * inverseInertiaTensorLocal.data[8];
  // by R*
  real data[9];
  data[0] = inverseInertiaTensorWorld.data[0] * transformationMatrix.data[0] +
            inverseInertiaTensorWorld.data[1] * transformationMatrix.data[1] +
            inverseInertiaTensorWorld.data[2] * transformationMatrix.data[2];
  data[1] = inverseInertiaTensorWorld.data[0] * transformationMatrix.data[4] +
            inverseInertiaTensorWorld.data[1] * transformationMatrix.data[5] +
            inverseInertiaTensorWorld.data[2] * transformationMatrix.data[6];
  data[2] = inverseInertiaTensorWorld.data[0] * transformationMatrix.data[8] +
            inverseInertiaTensorWorld.data[1] * transformationMatrix.data[9] +
            inverseInertiaTensorWorld.data[2] * transformationMatrix.data[10];

  data[3] = inverseInertiaTensorWorld.data[3] * transformationMatrix.data[0] +
            inverseInertiaTensorWorld.data[4] * transformationMatrix.data[1] +
            inverseInertiaTensorWorld.data[5] * transformationMatrix.data[2];
  data[4] = inverseInertiaTensorWorld.data[3] * transformationMatrix.data[4] +
            inverseInertiaTensorWorld.data[4] * transformationMatrix.data[5] +
            inverseInertiaTensorWorld.data[5] * transformationMatrix.data[6];
  data[5] = inverseInertiaTensorWorld.data[3] * transformationMatrix.data[8] +
            inverseInertiaTensorWorld.data[4] * transformationMatrix.data[9] +
            inverseInertiaTensorWorld.data[5] * transformationMatrix.data[10];

  data[6] = inverseInertiaTensorWorld.data[6] * transformationMatrix.data[0] +
            inverseInertiaTensorWorld.data[7] * transformationMatrix.data[1] +
            inverseInertiaTensorWorld.data[8] * transformationMatrix.data[2];
  data[7] = inverseInertiaTensorWorld.data[6] * transformationMatrix.data[4] +
            inverseInertiaTensorWorld.data[7] * transformationMatrix.data[5] +
            inverseInertiaTensorWorld.data[8] * transformationMatrix.data[6];
  data[8] = inverseInertiaTensorWorld.data[6] * transformationMatrix.data[8] +
            inverseInertiaTensorWorld.data[7] * transformationMatrix.data[9] +
            inverseInertiaTensorWorld.data[8] * transformationMatrix.data[10];

  inverseInertiaTensorWorld.data[0] = data[0];
  inverseInertiaTensorWorld.data[1] = data[1];
  inverseInertiaTensorWorld.data[2] = data[2];
  inverseInertiaTensorWorld.data[3] = data[3];
  inverseInertiaTensorWorld.data[4] = data[4];
  inverseInertiaTensorWorld.data[5] = data[5];
  inverseInertiaTensorWorld.data[6] = data[6];
  inverseInertiaTensorWorld.data[7] = data[7];
  inverseInertiaTensorWorld.data[8] = data[8];
}

} // namespace locusBody
