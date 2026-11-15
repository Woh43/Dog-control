// Local ROS-free extension of the pinned legged_control interface.
#include "legged_interface/SwingReferenceSettings.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace ocs2 {
namespace legged_robot {

SwingReferenceSettings loadSwingReferenceSettings(const std::string& fileName, const std::string& fieldName, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(fileName, pt);
  SwingReferenceSettings settings;
  const auto readScalar = [&](const std::string& key, scalar_t defaultValue) {
    const auto node = pt.get_child_optional(fieldName + "." + key);
    // An absent field is compatible; a present, malformed scalar must not silently fall back.
    return node ? node->get_value<scalar_t>() : defaultValue;
  };
  settings.flatGroundFootCenterHeight = readScalar("flatGroundFootCenterHeight", 0.0);
  settings.positionErrorGain = readScalar("positionErrorGain", 0.0);
  if (!std::isfinite(settings.flatGroundFootCenterHeight) || !std::isfinite(settings.positionErrorGain) ||
      settings.positionErrorGain < 0.0) {
    throw std::invalid_argument("[SwingReferenceSettings] foot-center height must be finite; positionErrorGain must be finite and >= 0");
  }
  if (verbose) {
    std::cerr << "[SwingReferenceSettings] configured flat foot-center world z=" << settings.flatGroundFootCenterHeight
              << " m, swing-only position error gain=" << settings.positionErrorGain << " 1/s" << std::endl;
  }
  return settings;
}

EndEffectorLinearConstraint::Config makeSwingNormalVelocityConstraintConfig(scalar_t zReference,
                                                                           scalar_t zVelocityReference,
                                                                           scalar_t positionErrorGain) {
  if (!std::isfinite(zReference) || !std::isfinite(zVelocityReference) || !std::isfinite(positionErrorGain) ||
      positionErrorGain < 0.0) {
    throw std::invalid_argument("[SwingReferenceSettings] non-finite reference or invalid position error gain");
  }
  EndEffectorLinearConstraint::Config config;
  config.b = vector_t::Constant(1, -zVelocityReference);
  config.Av = (matrix_t(1, 3) << 0.0, 0.0, 1.0).finished();
  if (positionErrorGain > 0.0) {
    config.b(0) -= positionErrorGain * zReference;
    config.Ax = (matrix_t(1, 3) << 0.0, 0.0, positionErrorGain).finished();
  }
  return config;
}

}  // namespace legged_robot
}  // namespace ocs2
