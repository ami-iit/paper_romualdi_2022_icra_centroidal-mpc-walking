/**
 * @file WholeBodyQPBlock.h
 * @authors Giulio Romualdi
 * @copyright This software may be modified and distributed under the terms of the GNU Lesser
 * General Public License v2.1 or any later version.
 */

#ifndef CENTROIDAL_MCP_WALKING_WHOLE_BODY_QP_BLOCK_H
#define CENTROIDAL_MCP_WALKING_WHOLE_BODY_QP_BLOCK_H

#include <string>
#include <map>
#include <memory>
#include <unordered_map>
#include <fstream>

#include <Eigen/Dense>

#include <BipedalLocomotion/ContactDetectors/FixedFootDetector.h>
#include <BipedalLocomotion/Contacts/Contact.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/CentroidalDynamics.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/FloatingBaseSystemKinematics.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/ForwardEuler.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/LinearTimeInvariantSystem.h>
#include <BipedalLocomotion/FloatingBaseEstimators/LeggedOdometry.h>
#include <BipedalLocomotion/FloatingBaseEstimators/ModelComputationsHelper.h>
#include <BipedalLocomotion/IK/CoMTask.h>
#include <BipedalLocomotion/IK/IntegrationBasedIK.h>
#include <BipedalLocomotion/IK/JointTrackingTask.h>
#include <BipedalLocomotion/IK/QPInverseKinematics.h>
#include <BipedalLocomotion/IK/SE3Task.h>
#include <BipedalLocomotion/IK/SO3Task.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/Planners/QuinticSpline.h>
#include <BipedalLocomotion/Planners/SwingFootPlanner.h>
#include <BipedalLocomotion/RobotInterface/YarpHelper.h>
#include <BipedalLocomotion/RobotInterface/YarpRobotControl.h>
#include <BipedalLocomotion/RobotInterface/YarpSensorBridge.h>
#include <BipedalLocomotion/SimplifiedModelControllers/CoMZMPController.h>
#include <BipedalLocomotion/System/Advanceable.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

#include <CentroidalMPCWalking/CentroidalMPCBlock.h>

namespace CentroidalMPCWalking
{
class WholeBodyQPBlock : public BipedalLocomotion::System::Advanceable<
                             BipedalLocomotion::ReducedModelControllers::CentroidalMPCState,
                             CentoidalMPCInput>
{

    typename WholeBodyQPBlock::Output m_output;
    typename WholeBodyQPBlock::Input m_input;

    std::string m_robot; /**< Robot name. */

    Eigen::VectorXd m_currentJointPos; /**< Current joint positions. */
    Eigen::VectorXd m_currentJointVel; /**< Current joint velocities. */
    Eigen::VectorXd m_desJointPos; /**< Current joint positions. */
    Eigen::VectorXd m_desJointVel; /**< Current joint velocities. */


    yarp::os::BufferedPort<yarp::sig::Vector> m_robotBasePort;

    manif::SE3d m_baseTransform;
    manif::SE3d::Tangent m_baseVelocity;

    BipedalLocomotion::RobotInterface::PolyDriverDescriptor m_controlBoard; /**< Control board
                                                                               remapper. */

    struct ContactWrenchHandler
    {
        BipedalLocomotion::RobotInterface::PolyDriverDescriptor polyDriverDescriptor;
        BipedalLocomotion::Math::Wrenchd wrench;
    };

    std::unordered_map<std::string, ContactWrenchHandler> m_leftFootContacWrenches;
    std::unordered_map<std::string, ContactWrenchHandler> m_rightFootContacWrenches;
    std::unordered_map<std::string, ContactWrenchHandler> m_externalContactWrenches;


    BipedalLocomotion::RobotInterface::YarpRobotControl m_robotControl; /**< Robot control object. */
    BipedalLocomotion::RobotInterface::YarpSensorBridge m_sensorBridge; /**< Sensor bridge object. */

    BipedalLocomotion::Estimators::LeggedOdometry m_floatingBaseEstimator;
    BipedalLocomotion::Contacts::FixedFootDetector m_fixedFootDetector;

    BipedalLocomotion::Planners::SwingFootPlanner m_leftFootPlanner;
    BipedalLocomotion::Planners::SwingFootPlanner m_rightFootPlanner;

    BipedalLocomotion::Planners::QuinticSpline m_rightFootAdjuster;
    BipedalLocomotion::Planners::QuinticSpline m_leftFootAdjuster;
    Eigen::Vector2d m_nextLeftFootPositionXY;
    Eigen::Vector2d m_nextRightFootPositionXY;

    BipedalLocomotion::SimplifiedModelControllers::CoMZMPController m_CoMZMPController;
    BipedalLocomotion::Contacts::ContactPhaseList m_contactPhaselist;
    BipedalLocomotion::Contacts::ContactListMap m_contactListMap;
    BipedalLocomotion::Contacts::ContactPhaseList::const_iterator m_phaseIt;

    struct InverseKinematicsAndTasks
    {
        std::shared_ptr<BipedalLocomotion::IK::IntegrationBasedIK> ik;
        std::shared_ptr<BipedalLocomotion::IK::SE3Task> leftFootTask;
        std::shared_ptr<BipedalLocomotion::IK::SO3Task> chestTask;
        std::shared_ptr<BipedalLocomotion::IK::SE3Task> rightFootTask;
        std::shared_ptr<BipedalLocomotion::IK::CoMTask> comTask;
        std::shared_ptr<BipedalLocomotion::IK::JointTrackingTask> regularizationTask;
    };
    InverseKinematicsAndTasks m_IKandTasks;

    struct CentroidalDynamicsIntegrator
    {
        std::shared_ptr<BipedalLocomotion::ContinuousDynamicalSystem::ForwardEuler<
            BipedalLocomotion::ContinuousDynamicalSystem::CentroidalDynamics>>
            integrator;
        std::shared_ptr<BipedalLocomotion::ContinuousDynamicalSystem::CentroidalDynamics>
            dynamics;
    };
    CentroidalDynamicsIntegrator m_centroidalSystem;

    struct CenterOfMassIntegrator
    {
        std::shared_ptr<BipedalLocomotion::ContinuousDynamicalSystem::ForwardEuler<
            BipedalLocomotion::ContinuousDynamicalSystem::LinearTimeInvariantSystem>>
            integrator;
        std::shared_ptr<BipedalLocomotion::ContinuousDynamicalSystem::LinearTimeInvariantSystem>
            dynamics;
    };
    CenterOfMassIntegrator m_comSystem;


    struct SystemAndIntegrator
    {
        std::shared_ptr<BipedalLocomotion::ContinuousDynamicalSystem::ForwardEuler<
            BipedalLocomotion::ContinuousDynamicalSystem::FloatingBaseSystemKinematics>>
            integrator;
        std::shared_ptr<BipedalLocomotion::ContinuousDynamicalSystem::FloatingBaseSystemKinematics>
            dynamics;
    };
    SystemAndIntegrator m_system;

    BipedalLocomotion::Estimators::KinDynComputationsDescriptor m_kinDynWithDesired;
    BipedalLocomotion::Estimators::KinDynComputationsDescriptor m_kinDynWithMeasured;


  std::ofstream m_myfile;
  double m_com0z;

  static constexpr double m_dT{0.01};
  double m_absoluteTime{0};
  bool m_isLeftAsjustedReady{false};
  bool m_isRightAsjustedReady{false};
  bool m_isLeftAdjusterAlreadyUsed{false};
  bool m_isRightAdjusterAlreadyUsed{false};

  bool createPolydriver(
      std::shared_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

  bool initializeRobotControl(
      std::shared_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

  bool instantiateSensorBridge(
      std::shared_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

  bool instantiateIK(
      std::shared_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

  bool instantiateLeggedOdometry(
      std::shared_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
      const std::string& modelPath,
      const std::vector<std::string>& jointLists);

  bool createKinDyn(const std::string& modelPath, const std::vector<std::string>& jointLists);

  bool updateFloatingBase();

  bool createAllContactWrenchesDriver(
      std::shared_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

  BipedalLocomotion::RobotInterface::PolyDriverDescriptor createContactWrenchDriver(
      std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
      const std::string& local);

  bool evaluateZMP(Eigen::Ref<Eigen::Vector2d> zmp);

  Eigen::Vector2d computeDesiredZMP(
      const std::map<std::string, BipedalLocomotion::Contacts::ContactWithCorners>& contacts);

  public:
  bool
  initialize(std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
      override;

  const Output& getOutput() const override;

  bool setInput(const Input& input) override;

  bool advance() override;

  bool isOutputValid() const override;

  bool close() override;
};
} // namespace System

#endif // CENTROIDAL_MCP_WALKING_WHOLE_BODY_QP_BLOCK_H
