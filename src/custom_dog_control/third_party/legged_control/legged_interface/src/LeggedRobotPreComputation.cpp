/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include "legged_interface/LeggedRobotPreComputation.h"
#include "legged_interface/SwingReferenceSettings.h"

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotPreComputation::LeggedRobotPreComputation(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                                     const SwingTrajectoryPlanner& swingTrajectoryPlanner, scalar_t swingPositionErrorGain)
    : pinocchioInterface_(std::move(pinocchioInterface)),
      info_(std::move(info)),
      swingTrajectoryPlannerPtr_(&swingTrajectoryPlanner),
      mappingPtr_(new CentroidalModelPinocchioMapping(info_)),
      swingPositionErrorGain_(swingPositionErrorGain) {
  // Validate even when the first precomputation request is not a constraint request.
  makeSwingNormalVelocityConstraintConfig(0.0, 0.0, swingPositionErrorGain_);
  eeNormalVelConConfigs_.resize(info_.numThreeDofContacts);
  mappingPtr_->setPinocchioInterface(pinocchioInterface_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotPreComputation::LeggedRobotPreComputation(const LeggedRobotPreComputation& rhs)
    : pinocchioInterface_(rhs.pinocchioInterface_),
      info_(rhs.info_),
      swingTrajectoryPlannerPtr_(rhs.swingTrajectoryPlannerPtr_),
      mappingPtr_(rhs.mappingPtr_->clone()),
      swingPositionErrorGain_(rhs.swingPositionErrorGain_) {
  eeNormalVelConConfigs_.resize(rhs.eeNormalVelConConfigs_.size());
  mappingPtr_->setPinocchioInterface(pinocchioInterface_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotPreComputation::request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) {
  if (!request.containsAny(Request::Cost + Request::Constraint + Request::SoftConstraint)) {
    return;
  }

  // lambda to set config for normal velocity constraints
  auto eeNormalVelConConfig = [&](size_t footIndex) {
    return makeSwingNormalVelocityConstraintConfig(swingTrajectoryPlannerPtr_->getZpositionConstraint(footIndex, t),
                                                   swingTrajectoryPlannerPtr_->getZvelocityConstraint(footIndex, t),
                                                   swingPositionErrorGain_);
  };

  if (request.contains(Request::Constraint)) {
    for (size_t i = 0; i < info_.numThreeDofContacts; i++) {
      eeNormalVelConConfigs_[i] = eeNormalVelConConfig(i);
    }
  }

  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  vector_t q = mappingPtr_->getPinocchioJointPosition(x);
  if (request.contains(Request::Approximation)) {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::updateGlobalPlacements(model, data);
    pinocchio::computeJointJacobians(model, data);

    updateCentroidalDynamics(pinocchioInterface_, info_, q);
    vector_t v = mappingPtr_->getPinocchioJointVelocity(x, u);
    updateCentroidalDynamicsDerivatives(pinocchioInterface_, info_, q, v);
  } else {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
  }
}

}  // namespace legged_robot
}  // namespace ocs2
