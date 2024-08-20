/**
 * @file CentoidalMPCBlock.h
 * @authors Giulio Romualdi
 * @copyright This software may be modified and distributed under the terms of the BSD-3-Clause license.
 */

#ifndef CENTROIDAL_MCP_WALKING_CENTROIDAL_MCP_BLOCK_H
#define CENTROIDAL_MCP_WALKING_CENTROIDAL_MCP_BLOCK_H

#include <map>
#include <memory>
#include <string>

#include <Eigen/Dense>

#include <BipedalLocomotion/Contacts/Contact.h>
#include <BipedalLocomotion/Contacts/ContactPhaseList.h>
#include <BipedalLocomotion/FloatingBaseEstimators/LeggedOdometry.h>
#include <BipedalLocomotion/IK/CoMTask.h>
#include <BipedalLocomotion/IK/IntegrationBasedIK.h>
#include <BipedalLocomotion/IK/JointTrackingTask.h>
#include <BipedalLocomotion/IK/QPInverseKinematics.h>
#include <BipedalLocomotion/IK/SE3Task.h>
#include <BipedalLocomotion/Math/Wrench.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ReducedModelControllers/CentroidalMPC.h>
#include <BipedalLocomotion/System/Advanceable.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

namespace CentroidalMPCWalking
{

struct CentoidalMPCInput
{
    Eigen::Vector3d com;
    Eigen::Vector3d dcom;
    Eigen::Vector3d angularMomentum;

    BipedalLocomotion::Math::Wrenchd totalExternalWrench;

    manif::SE3d leftFoot;
    manif::SE3d rightFoot;

    bool isValid{false};
};

} // namespace CentroidalMPCWalking

namespace CentroidalMPCWalking
{
class CentroidalMPCBlock : public BipedalLocomotion::System::Advanceable<
                               CentoidalMPCInput,
                               BipedalLocomotion::ReducedModelControllers::CentroidalMPCState>
{
    typename CentroidalMPCBlock::Output m_output;

    BipedalLocomotion::ReducedModelControllers::CentroidalMPC m_controller;

    BipedalLocomotion::Contacts::ContactPhaseList m_phaseList;

    bool m_inputValid{false};
    double m_currentTime{0};
    BipedalLocomotion::Contacts::ContactPhaseList::const_iterator m_phaseIt;
    bool m_isFirstRun{true};
    Eigen::MatrixXd m_comTraj;
    unsigned int m_indexCoM{0};

public:
    bool initialize(std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler>
                        handler) override;

    const Output& getOutput() const override;

    bool setInput(const Input& input) override;

    bool advance() override;

    bool isOutputValid() const override;
};
} // namespace CentroidalMPCWalking

#endif // CENTROIDAL_MCP_WALKING_CENTROIDAL_MCP_BLOCK_H
