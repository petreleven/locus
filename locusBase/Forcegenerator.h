#include "Core.h"
#include "LocusMathFunctions.h"
#include "RigidBody.h"

using locus::real;
using locusBody::RigidBody;

class ForceGenerator {
public:
  virtual void applyForce(RigidBody *rigidBody, real dt) const = 0;
};

class GravityForceGenerator : public ForceGenerator {
  locus::Vector3 gravity;

public:
  GravityForceGenerator(const locus::Vector3 &g) : gravity(g){};
  virtual void applyForce(RigidBody *rigidBody, real dt) const;
};

class SpringForceGenerator : public ForceGenerator {
  real springConstant;
  real restLength;
  locus::Vector3 suspensionPoint;
  locus::Vector3 connectionPoint;
  RigidBody *other;

public:
  /*
  Creates a spring force
  @param - locus::real - Spring Constant
  @param - locus::real - restLength
  @param - locus::Vector3 - Suspension local space of springBody;
  @param - locus::Vector3 - Connection Point in local space of suspended body
  @param - locusBody::RigidBody *  - Rigidbody where spring is suspended from
  */
  SpringForceGenerator(const real &k, const real &length,
                       const locus::Vector3 p, locus::Vector3 c,
                       RigidBody *other)
      : springConstant(k), restLength(length), suspensionPoint(p),
        connectionPoint(c), other(other){};
  /*
  Apply spring force to a rigid body
  @param - locusBody::RigidBody *  - RigidBody hanging from spring
  @param - locus::Real - timestep
  */
  virtual void applyForce(RigidBody *body, const real &dt) const;
};
