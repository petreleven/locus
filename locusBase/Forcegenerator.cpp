#include "Forcegenerator.h"
#include "Core.h"
#include "LocusMathFunctions.h"
#include <cmath>

using locusMath::LMathFunctions;

void GravityForceGenerator::applyForce(RigidBody *rigidBody, real dt) const {
  if (rigidBody->hasInfiniteMass()) {
    return;
  }
  rigidBody->addForce(gravity);
}

void SpringForceGenerator::applyForce(RigidBody *body, const real &dt) const {
  locus::Vector3 suspensionPointWorld = other->getPointWorld(suspensionPoint);
  locus::Vector3 connectionPointWorld = body->getPointWorld(connectionPoint);
  locus::Vector3 diff = (connectionPoint - suspensionPoint);
  real dx = LMathFunctions::Magnitude(diff);
  // f = -kx;
  real forceMag = -springConstant * std::abs(dx - restLength);
  locus::Vector3 forceAndDir = LMathFunctions::Normalize(diff);
  forceAndDir *= forceMag;
  body->addForceAtPoint(forceAndDir, connectionPointWorld);
}
