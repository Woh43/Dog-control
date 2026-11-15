// Local ROS-free extension of the pinned legged_control interface.
#pragma once

#include <string>

#include "legged_interface/constraint/EndEffectorLinearConstraint.h"

namespace ocs2 {
namespace legged_robot {

/** A configured horizontal plane for foot-frame origins, not a measured terrain estimate. */
struct SwingReferenceSettings {
  // World z of the foot frame at contact. For spherical feet on z=0 ground this is the radius.
  scalar_t flatGroundFootCenterHeight = 0.0;
  // Only active in the swing normal-velocity constraint; independent of stance positionErrorGain.
  // Units: 1/s. Zero preserves velocity-only tracking.
  scalar_t positionErrorGain = 0.0;
};

SwingReferenceSettings loadSwingReferenceSettings(const std::string& fileName,
                                                  const std::string& fieldName = "swing_reference", bool verbose = true);

/** Build vz - vz_ref + gain * (z - z_ref) = 0. Position in m, velocity in m/s, gain in 1/s. */
EndEffectorLinearConstraint::Config makeSwingNormalVelocityConstraintConfig(scalar_t zReference,
                                                                           scalar_t zVelocityReference,
                                                                           scalar_t positionErrorGain);

}  // namespace legged_robot
}  // namespace ocs2
