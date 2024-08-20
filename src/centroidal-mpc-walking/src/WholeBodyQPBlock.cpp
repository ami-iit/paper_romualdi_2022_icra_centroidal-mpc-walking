/**
 * @file WholeBodyQPBlock.cpp
 * @authors Giulio Romualdi
 * @copyright This software may be modified and distributed under the terms of the BSD-3-Clause license.
 */

#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>

#include <Eigen/Dense>

#include <manif/SE3.h>

#include <iDynTree/Core/SpatialMomentum.h>
#include <iDynTree/Model/Model.h>

#include <yarp/os/RFModule.h>
#include <yarp/sig/Vector.h>

#include <BipedalLocomotion/ContactDetectors/FixedFootDetector.h>
#include <BipedalLocomotion/Contacts/ContactListJsonParser.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/LinearTimeInvariantSystem.h>
#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/FloatingBaseEstimators/LeggedOdometry.h>
#include <BipedalLocomotion/FloatingBaseEstimators/ModelComputationsHelper.h>
#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/Math/Wrench.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/RobotInterface/YarpHelper.h>
#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/System/VariablesHandler.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <CentroidalMPCWalking/Utilities.h>
#include <CentroidalMPCWalking/WholeBodyQPBlock.h>

using namespace CentroidalMPCWalking;
using namespace BipedalLocomotion::ParametersHandler;

void updateContactPhaseList(
    const std::map<std::string, BipedalLocomotion::Contacts::PlannedContact>& nextPlannedContacts,
    BipedalLocomotion::Contacts::ContactListMap& contactListMap)
{
    for (const auto& [key, contact] : nextPlannedContacts)
    {
        auto it = contactListMap.at(key).getPresentContact(contact.activationTime);
        contactListMap.at(key).editContact(it, contact);
    }
}

bool WholeBodyQPBlock::createKinDyn(const std::string& modelPath,
                                    const std::vector<std::string>& jointLists)
{
    constexpr auto errorPrefix = "[WholeBodyQPBlock::initializeLeggedOdometry]";

    auto handlerKinDyn = std::make_shared<StdImplementation>();
    handlerKinDyn->setParameter("joints_list", jointLists);
    handlerKinDyn->setParameter("model_file_name", modelPath);

    m_kinDynWithDesired
        = BipedalLocomotion::Estimators::constructKinDynComputationsDescriptor(handlerKinDyn);

    m_kinDynWithMeasured
        = BipedalLocomotion::Estimators::constructKinDynComputationsDescriptor(handlerKinDyn);

    if (!m_kinDynWithDesired.isValid() || !m_kinDynWithMeasured.isValid())
    {
        BipedalLocomotion::log()->error("{} Unable to load a KinDynComputation object",
                                        errorPrefix);
        return false;
    }

    return true;
}

bool WholeBodyQPBlock::instantiateLeggedOdometry(std::shared_ptr<const IParametersHandler> handler,
                                                 const std::string& modelPath,
                                                 const std::vector<std::string>& jointLists)
{
    constexpr auto logPrefix = "[WholeBodyQPBlock::instantiateLeggedOdometry]";

    auto handlerKinDyn = std::make_shared<StdImplementation>();
    handlerKinDyn->setParameter("joints_list", jointLists);
    handlerKinDyn->setParameter("model_file_name", modelPath);

    auto kinDyn
        = BipedalLocomotion::Estimators::constructKinDynComputationsDescriptor(handlerKinDyn);

    if (!m_floatingBaseEstimator.initialize(handler->getGroup("FLOATING_BASE_ESTIMATOR"),
                                            kinDyn.kindyn))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the legged odometry.", logPrefix);
        return false;
    }

    if (!m_fixedFootDetector.initialize(handler->getGroup("FIXED_FOOT_DETECTOR")))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the fixed foot detector.",
                                        logPrefix);
        return false;
    }

    return true;
}

bool WholeBodyQPBlock::instantiateIK(std::shared_ptr<const IParametersHandler> handler)
{
    using namespace BipedalLocomotion::IK;
    constexpr auto logError = "[WholeBodyQPBlock::instantiateIK]";

    Eigen::VectorXd weightRegularization(m_kinDynWithDesired.kindyn->getNrOfDegreesOfFreedom());

    handler->getGroup("REGULARIZATION_TASK").lock()->getParameter("weight", weightRegularization);

    Eigen::Vector3d weightChest;

    handler->getGroup("CHEST_TASK").lock()->getParameter("weight", weightChest);

    BipedalLocomotion::System::VariablesHandler variables;
    variables.addVariable("robotVelocity",
                          m_kinDynWithDesired.kindyn->getNrOfDegreesOfFreedom() + 6);

    constexpr std::size_t highPriority = 0;
    constexpr std::size_t lowPriority = 1;

    m_IKandTasks.ik = std::make_shared<QPInverseKinematics>();
    if (!m_IKandTasks.ik->initialize(handler))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the IK.", logError);
        return false;
    }

    m_IKandTasks.leftFootTask = std::make_shared<SE3Task>();
    if (!m_IKandTasks.leftFootTask->setKinDyn(m_kinDynWithDesired.kindyn))
    {
        BipedalLocomotion::log()->error("{} Unable to set the kinDynObject in the left foot.",
                                        logError);
        return false;
    }
    if (!m_IKandTasks.leftFootTask->initialize(handler->getGroup("L_FOOT")))
    {
        BipedalLocomotion::log()->error("{} Unable initialize in the left foot.", logError);
        return false;
    }
    if (!m_IKandTasks.ik->addTask(m_IKandTasks.leftFootTask, "l_foot_task", highPriority))
    {
        BipedalLocomotion::log()->error("{} Unable to add the left foot task.", logError);
        return false;
    }

    m_IKandTasks.rightFootTask = std::make_shared<SE3Task>();
    if (!m_IKandTasks.rightFootTask->setKinDyn(m_kinDynWithDesired.kindyn))
    {
        BipedalLocomotion::log()->error("{} Unable to set the kinDynObject in the right foot.",
                                        logError);
        return false;
    }
    if (!m_IKandTasks.rightFootTask->initialize(handler->getGroup("R_FOOT")))
    {
        BipedalLocomotion::log()->error("{} Unable initialize in the right foot.", logError);
        return false;
    }
    if (!m_IKandTasks.ik->addTask(m_IKandTasks.rightFootTask, "r_foot_task", highPriority))
    {
        BipedalLocomotion::log()->error("{} Unable to add the right foot task.", logError);
        return false;
    }

    //// com
    m_IKandTasks.comTask = std::make_shared<CoMTask>();
    if (!m_IKandTasks.comTask->setKinDyn(m_kinDynWithDesired.kindyn))
    {
        BipedalLocomotion::log()->error("{} Unable to set the kinDynObject for the com task.",
                                        logError);
        return false;
    }
    if (!m_IKandTasks.comTask->initialize(handler->getGroup("COM_TASK")))
    {
        BipedalLocomotion::log()->error("{} Unable initialize in the com task.", logError);
        return false;
    }
    if (!m_IKandTasks.ik->addTask(m_IKandTasks.comTask, "com", highPriority))
    {
        BipedalLocomotion::log()->error("{} Unable to add the com task.", logError);
        return false;
    }

    //// CHEST
    m_IKandTasks.chestTask = std::make_shared<SO3Task>();
    if (!m_IKandTasks.chestTask->setKinDyn(m_kinDynWithDesired.kindyn))
    {
        BipedalLocomotion::log()->error("{} Unable to set the kinDynObject for the com task.",
                                        logError);
        return false;
    }
    if (!m_IKandTasks.chestTask->initialize(handler->getGroup("CHEST_TASK")))
    {
        BipedalLocomotion::log()->error("{} Unable initialize in the com task.", logError);
        return false;
    }
    if (!m_IKandTasks.ik->addTask(m_IKandTasks.chestTask, "chest", lowPriority, weightChest))
    {
        BipedalLocomotion::log()->error("{} Unable to add the com task.", logError);
        return false;
    }

    m_IKandTasks.regularizationTask = std::make_shared<JointTrackingTask>();

    if (!m_IKandTasks.regularizationTask->setKinDyn(m_kinDynWithDesired.kindyn))
    {
        BipedalLocomotion::log()->error("{} Unable to set the kinDynObject for the com task.",
                                        logError);
        return false;
    }
    if (!m_IKandTasks.regularizationTask->initialize(handler->getGroup("REGULARIZATION_TASK")))
    {
        BipedalLocomotion::log()->error("{} Unable initialize in the com task.", logError);
        return false;
    }

    if (!m_IKandTasks.ik->addTask(m_IKandTasks.regularizationTask,
                                  "regularization",
                                  lowPriority,
                                  weightRegularization))
    {
        BipedalLocomotion::log()->error("{} Unable to add the com task.", logError);
        return false;
    }

    if (!m_IKandTasks.ik->finalize(variables))
    {
        BipedalLocomotion::log()->error("{} Unable to finalize the IK.", logError);
        return false;
    }

    return true;
}

bool WholeBodyQPBlock::initializeRobotControl(std::shared_ptr<const IParametersHandler> handler)
{
    constexpr auto errorPrefix = "[WholeBodyQPBlock::initializeRobotControl]";

    if (!m_robotControl.initialize(handler->getGroup("ROBOT_CONTROL")))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the robot control.", errorPrefix);
        return false;
    }
    if (!m_robotControl.setDriver(m_controlBoard.poly))
    {
        BipedalLocomotion::log()->error("{} Unable to set the polydriver.", errorPrefix);
        return false;
    }

    return true;
}

bool WholeBodyQPBlock::instantiateSensorBridge(std::shared_ptr<const IParametersHandler> handler)
{
    constexpr auto errorPrefix = "[WholeBodyQPBlock::instantiateSensorBridge]";

    if (!m_sensorBridge.initialize(handler->getGroup("SENSOR_BRIDGE")))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the sensor bridge.", errorPrefix);
        return false;
    }

    yarp::dev::PolyDriverList list;
    list.push(m_controlBoard.poly.get(), m_controlBoard.key.c_str());
    for (auto& [key, driver] : m_leftFootContacWrenches)
    {
        list.push(driver.polyDriverDescriptor.poly.get(), key.c_str());
    }

    for (auto& [key, driver] : m_rightFootContacWrenches)
    {
        list.push(driver.polyDriverDescriptor.poly.get(), key.c_str());
    }

    for (auto& [key, driver] : m_externalContactWrenches)
    {
        list.push(driver.polyDriverDescriptor.poly.get(), key.c_str());
    }

    if (!m_sensorBridge.setDriversList(list))
    {
        BipedalLocomotion::log()->error("{} Unable to set the driver list.", errorPrefix);
        return false;
    }

    return true;
}

bool WholeBodyQPBlock::updateFloatingBase()
{
    constexpr auto logPrefix = "[WholeBodyQPBlock::updateFloatingBase]";

    if (!m_floatingBaseEstimator.setKinematics(m_currentJointPos, m_currentJointVel))
    {
        BipedalLocomotion::log()->error("{} Unable to set the kinematics in the base estimator.",
                                        logPrefix);
        return false;
    }

    for (const auto& [key, foot] : m_fixedFootDetector.getOutput())
    {
        // TODO Please change the signature of setContactStatus
        const auto frameName = m_kinDynWithDesired.kindyn->model().getFrameName(foot.index);
        if (!m_floatingBaseEstimator.setContactStatus(frameName,
                                                      foot.isActive,
                                                      foot.switchTime,
                                                      foot.lastUpdateTime))
        {
            BipedalLocomotion::log()->error("{} Unable to set the contact status in the base "
                                            "estimator.",
                                            logPrefix);
            return false;
        }
    }

    if (!m_floatingBaseEstimator.advance())
    {
        BipedalLocomotion::log()->error("{} Unable to update the floating base estimator.",
                                        logPrefix);
        return false;
    }

    if (!m_floatingBaseEstimator
             .changeFixedFrame(m_fixedFootDetector.getFixedFoot().index,
                               m_fixedFootDetector.getFixedFoot().pose.quat(),
                               m_fixedFootDetector.getFixedFoot().pose.translation()))
    {
        BipedalLocomotion::log()->error("{} Unable to change the fixed frame in the base "
                                        "estimator.",
                                        logPrefix);
        return false;
    }

    return true;
}

bool WholeBodyQPBlock::createPolydriver(std::shared_ptr<const IParametersHandler> handler)
{
    constexpr auto errorPrefix = "[WholeBodyQPBlock::createPolydriver]";

    std::string name;
    if (!handler->getParameter("name", name))
    {
        BipedalLocomotion::log()->error("{} Unable to find the name.", errorPrefix);
        return false;
    }

    auto ptr = handler->getGroup("ROBOT_INTERFACE").lock();
    if (ptr == nullptr)
    {
        BipedalLocomotion::log()->error("{} Robot interface options is empty.", errorPrefix);
        return false;
    }
    ptr->setParameter("local_prefix", name);
    m_controlBoard = BipedalLocomotion::RobotInterface::constructRemoteControlBoardRemapper(ptr);
    if (!m_controlBoard.isValid())
    {
        BipedalLocomotion::log()->error("{} the robot polydriver has not been constructed.",
                                        errorPrefix);
        return false;
    }

    return true;
}

BipedalLocomotion::RobotInterface::PolyDriverDescriptor
WholeBodyQPBlock::createContactWrenchDriver(std::weak_ptr<const IParametersHandler> handler,
                                            const std::string& local)
{
    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        return BipedalLocomotion::RobotInterface::PolyDriverDescriptor();
    }
    auto tmp = ptr->clone();
    tmp->setParameter("local_prefix", local);
    return BipedalLocomotion::RobotInterface::constructGenericSensorClient(tmp);
}

bool WholeBodyQPBlock::createAllContactWrenchesDriver(
    std::shared_ptr<const IParametersHandler> handler)
{
    constexpr auto errorPrefix = "[WholeBodyQPBlock::createGenericSensorClient]";

    std::string localPrefix;
    if (!handler->getParameter("name", localPrefix))
    {
        BipedalLocomotion::log()->error("{} Unable to find the name.", errorPrefix);
        return false;
    }

    auto ptr = handler->getGroup("CONTACT_WRENCHES").lock();
    if (ptr == nullptr)
    {
        BipedalLocomotion::log()->error("{} Robot interface options is empty.", errorPrefix);
        return false;
    }

    std::vector<std::string> contactWrenchClients;
    if (!ptr->getParameter("left_contact_wrenches_group", contactWrenchClients))
    {
        BipedalLocomotion::log()->error("{} Unable to find the left contact wrench group.",
                                        errorPrefix);
        return false;
    }

    for (const auto& wrench : contactWrenchClients)
    {
        BipedalLocomotion::RobotInterface::PolyDriverDescriptor descriptor
            = this->createContactWrenchDriver(ptr->getGroup(wrench), localPrefix);

        if (!descriptor.isValid())
        {
            BipedalLocomotion::log()->error("{} The generic sensor client for the wrench named {} "
                                            "cannot be opened.",
                                            errorPrefix,
                                            wrench);
            return false;
        }

        m_leftFootContacWrenches[descriptor.key].polyDriverDescriptor = descriptor;
    }

    if (!ptr->getParameter("right_contact_wrenches_group", contactWrenchClients))
    {
        BipedalLocomotion::log()->error("{} Unable to find the right contact wrench group.",
                                        errorPrefix);
        return false;
    }

    for (const auto& wrench : contactWrenchClients)
    {
        BipedalLocomotion::RobotInterface::PolyDriverDescriptor descriptor
            = this->createContactWrenchDriver(ptr->getGroup(wrench), localPrefix);

        if (!descriptor.isValid())
        {
            BipedalLocomotion::log()->error("{} The generic sensor client for the wrench named {} "
                                            "cannot be opened.",
                                            errorPrefix,
                                            wrench);
            return false;
        }

        m_rightFootContacWrenches[descriptor.key].polyDriverDescriptor = descriptor;
    }

    if (!ptr->getParameter("external_contact_wrenches_group", contactWrenchClients))
    {
        BipedalLocomotion::log()->error("{} Unable to find the right contact wrench group.",
                                        errorPrefix);
        return false;
    }

    for (const auto& wrench : contactWrenchClients)
    {
        BipedalLocomotion::RobotInterface::PolyDriverDescriptor descriptor
            = this->createContactWrenchDriver(ptr->getGroup(wrench), localPrefix);

        if (!descriptor.isValid())
        {
            BipedalLocomotion::log()->error("{} The generic sensor client for the wrench named {} "
                                            "cannot be opened.",
                                            errorPrefix,
                                            wrench);
            return false;
        }

        m_externalContactWrenches[descriptor.key].polyDriverDescriptor = descriptor;
    }

    return true;
}

bool WholeBodyQPBlock::initialize(std::weak_ptr<const IParametersHandler> handler)
{
    constexpr auto logPrefix = "[WholeBodyQPBlock::initialize]";

    auto parametersHandler = handler.lock();

    BipedalLocomotion::log()->info("{} Create the polydriver.", logPrefix);
    if (!this->createPolydriver(parametersHandler))
    {
        BipedalLocomotion::log()->error("{} Unable to create the polydriver.", logPrefix);
        return false;
    }
    if (!this->createAllContactWrenchesDriver(parametersHandler))
    {
        BipedalLocomotion::log()->error("{} Unable to create the contact wrench drivers.",
                                        logPrefix);
        return false;
    }

    // TODO remove me
    std::this_thread::sleep_for(std::chrono::duration<double>(1));

    BipedalLocomotion::log()->info("{} Create the robot control helper.", logPrefix);
    if (!this->initializeRobotControl(parametersHandler))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the robotControl interface.",
                                        logPrefix);
        return false;
    }

    BipedalLocomotion::log()->info("{} Create the sensor bridge.", logPrefix);
    if (!this->instantiateSensorBridge(parametersHandler))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the sensor bridge.", logPrefix);
        return false;
    }

    std::string name;
    if (!parametersHandler->getParameter("name", name))
    {
        BipedalLocomotion::log()->error("{} Unable to find the name.", logPrefix);
        return false;
    }

    if (!this->createKinDyn(yarp::os::ResourceFinder::getResourceFinderSingleton().findFileByName(
                                "model.urdf"),
                            m_robotControl.getJointList()))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the kinDyn.", logPrefix);
        return false;
    }

    m_kinDynWithDesired.kindyn->setFloatingBase("root_link");
    m_kinDynWithMeasured.kindyn->setFloatingBase("root_link");

    if (!this->instantiateIK(parametersHandler->getGroup("IK").lock()))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the IK.", logPrefix);
        return false;
    }

    if (!this->instantiateLeggedOdometry(parametersHandler,
                                         yarp::os::ResourceFinder::getResourceFinderSingleton()
                                             .findFileByName("model.urdf"),
                                         m_robotControl.getJointList()))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the legged odometry.", logPrefix);
        return false;
    }

    if (!this->m_CoMZMPController.initialize(parametersHandler->getGroup("COM_ZMP_CONTROLLER")))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the CoM-ZMP Controller.",
                                        logPrefix);
        return false;
    }

    ///
    // Reset kinDynObject and floating base integrator
    const auto numberOfJoints = m_robotControl.getJointList().size();

    m_currentJointPos.resize(numberOfJoints);
    m_currentJointVel.resize(numberOfJoints);
    m_currentJointVel.setZero();

    m_desJointPos.resize(numberOfJoints);
    m_desJointVel.resize(numberOfJoints);
    m_desJointVel.setZero();

    if (!m_sensorBridge.advance())
    {
        BipedalLocomotion::log()->error("{} Unable to get the robot state.", logPrefix);
        return false;
    }
    m_sensorBridge.getJointPositions(m_currentJointPos);
    m_sensorBridge.getJointPositions(m_desJointPos);

    constexpr double scaling = 1.0;
    constexpr double scalingPos = 1.5;
    constexpr double scalingPosY = 0;
    // // t  0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19 20  21
    // 22  23  24  25  26  27  28  29
    // // L
    // |+++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|++++++++++|---|+++++++++++|
    // // R
    // |+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|++++++++++|---|+++|

    Eigen::Vector3d leftPosition = Eigen::Vector3d::Zero();
    leftPosition(1) = 0.08;
    manif::SE3d leftTransform(leftPosition, manif::SO3d::Identity());

    m_contactListMap["left_foot"].setDefaultIndex(
        m_kinDynWithMeasured.kindyn->model().getFrameIndex("l_sole"));
    m_contactListMap["left_foot"].addContact(leftTransform, 0.0, 1.0 * scaling);

    leftPosition(0) += 0.05 * scalingPos;
    leftTransform.translation(leftPosition);
    m_contactListMap["left_foot"].addContact(leftTransform, 2.0 * scaling, 5.0 * scaling);

    leftPosition(0) += 0.1 * scalingPos;
    // leftTransform.quat(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX()));
    leftTransform.translation(leftPosition);
    m_contactListMap["left_foot"].addContact(leftTransform, 6.0 * scaling, 9.0 * scaling);

    leftPosition(0) += 0.1 * scalingPos;
    leftTransform.quat(manif::SO3d::Identity());
    leftTransform.translation(leftPosition);
    m_contactListMap["left_foot"].addContact(leftTransform, 10.0 * scaling, 13.0 * scaling);

    leftPosition(0) += 0.1 * scalingPos;
    leftTransform.translation(leftPosition);
    m_contactListMap["left_foot"].addContact(leftTransform, 14.0 * scaling, 17.0 * scaling);

    leftPosition(1) -= 0.01 * scalingPosY;
    leftTransform.translation(leftPosition);
    m_contactListMap["left_foot"].addContact(leftTransform, 18.0 * scaling, 21.0 * scaling);

    leftPosition(1) -= 0.01 * scalingPosY;
    leftTransform.translation(leftPosition);
    m_contactListMap["left_foot"].addContact(leftTransform, 22.0 * scaling, 25.0 * scaling);

    leftPosition(1) -= 0.01 * scalingPosY;
    leftTransform.translation(leftPosition);
    m_contactListMap["left_foot"].addContact(leftTransform, 26.0 * scaling, 29.0 * scaling);

    // right foot
    // first footstep
    Eigen::Vector3d rightPosition = Eigen::Vector3d::Zero();
    rightPosition(1) = -0.08;
    manif::SE3d rightTransform(rightPosition, manif::SO3d::Identity());
    m_contactListMap["right_foot"].setDefaultIndex(
        m_kinDynWithMeasured.kindyn->model().getFrameIndex("r_sole"));
    m_contactListMap["right_foot"].addContact(rightTransform, 0.0, 3.0 * scaling);

    rightPosition(0) += 0.1 * scalingPos;
    rightTransform.translation(rightPosition);
    m_contactListMap["right_foot"].addContact(rightTransform, 4.0 * scaling, 7.0 * scaling);

    rightPosition(0) += 0.1 * scalingPos;
    rightTransform.translation(rightPosition);
    m_contactListMap["right_foot"].addContact(rightTransform, 8.0 * scaling, 11.0 * scaling);

    rightPosition(0) += 0.1 * scalingPos;
    rightTransform.translation(rightPosition);
    m_contactListMap["right_foot"].addContact(rightTransform, 12.0 * scaling, 15.0 * scaling);

    rightPosition(0) += 0.05 * scalingPos;
    rightPosition(1) -= 0.01 * scalingPosY;
    rightTransform.translation(rightPosition);
    m_contactListMap["right_foot"].addContact(rightTransform, 16.0 * scaling, 19.0 * scaling);

    rightPosition(1) -= 0.01 * scalingPosY;
    rightTransform.translation(rightPosition);
    m_contactListMap["right_foot"].addContact(rightTransform, 20.0 * scaling, 23.0 * scaling);

    rightPosition(1) -= 0.01 * scalingPosY;
    rightTransform.translation(rightPosition);
    m_contactListMap["right_foot"].addContact(rightTransform, 24.0 * scaling, 27.0 * scaling);

    rightPosition(1) -= 0.01 * scalingPosY;
    rightTransform.translation(rightPosition);
    m_contactListMap["right_foot"].addContact(rightTransform, 28.0 * scaling, 29.0 * scaling);

    // contact phase list
    // m_contactListMap = BipedalLocomotion::Contacts::m_contactListMapFromJson("footsteps.json");
    m_contactPhaselist.setLists(m_contactListMap);
    m_fixedFootDetector.setContactPhaseList(m_contactPhaselist);
    m_phaseIt = m_contactPhaselist.getPresentPhase(m_absoluteTime);

    // feet
    std::shared_ptr<IParametersHandler> a = std::make_shared<StdImplementation>();
    a->setParameter("sampling_time", m_dT);
    a->setParameter("step_height", 0.03);
    a->setParameter("foot_apex_time", 0.5);
    a->setParameter("foot_landing_velocity", 0.0);
    a->setParameter("foot_landing_acceleration", 0.0);

    m_leftFootPlanner.initialize(a);
    m_rightFootPlanner.initialize(a);

    m_leftFootPlanner.setContactList(m_contactListMap["left_foot"]);
    m_rightFootPlanner.setContactList(m_contactListMap["right_foot"]);

    m_leftFootAdjuster.setAdvanceTimeStep(m_dT);
    m_rightFootAdjuster.setAdvanceTimeStep(m_dT);
    m_leftFootAdjuster.setFinalConditions(Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero());
    m_rightFootAdjuster.setFinalConditions(Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero());

    if (!this->updateFloatingBase())
    {
        BipedalLocomotion::log()->error("{} Unable to update the floating base.", logPrefix);
        return false;
    }

    m_baseTransform = m_floatingBaseEstimator.getOutput().basePose;
    m_baseVelocity = m_floatingBaseEstimator.getOutput().baseTwist;

    Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
    gravity(2) = -BipedalLocomotion::Math::StandardAccelerationOfGravitation;

    m_kinDynWithMeasured.kindyn->setRobotState(m_baseTransform.transform(),
                                               m_currentJointPos,
                                               iDynTree::make_span(m_baseVelocity.data(),
                                                                   manif::SE3d::Tangent::DoF),
                                               m_currentJointVel,
                                               gravity);

    /////// update kinDyn
    m_kinDynWithDesired.kindyn->setRobotState(m_baseTransform.transform(),
                                              m_desJointPos,
                                              iDynTree::make_span(m_baseVelocity.data(),
                                                                  manif::SE3d::Tangent::DoF),
                                              m_desJointVel,
                                              gravity);

    m_IKandTasks.regularizationTask->setSetPoint(m_desJointPos);

    using namespace BipedalLocomotion::ContinuousDynamicalSystem;
    m_system.dynamics = std::make_shared<FloatingBaseSystemKinematics>();
    m_system.dynamics->setState(
        {m_baseTransform.translation(), m_baseTransform.asSO3(), m_desJointPos});

    m_system.integrator = std::make_shared<ForwardEuler<FloatingBaseSystemKinematics>>();
    m_system.integrator->setIntegrationStep(m_dT);
    m_system.integrator->setDynamicalSystem(m_system.dynamics);

    ////// centroidal system
    m_centroidalSystem.dynamics = std::make_shared<CentroidalDynamics>();
    m_centroidalSystem.dynamics->setState(
        {iDynTree::toEigen(m_kinDynWithDesired.kindyn->getCenterOfMassPosition()),
         Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero()});

    m_com0z = m_kinDynWithDesired.kindyn->getCenterOfMassPosition()(2);

    m_centroidalSystem.integrator = std::make_shared<ForwardEuler<CentroidalDynamics>>();
    m_centroidalSystem.integrator->setIntegrationStep(m_dT);
    m_centroidalSystem.integrator->setDynamicalSystem(m_centroidalSystem.dynamics);

    m_comSystem.dynamics = std::make_shared<LinearTimeInvariantSystem>();
    m_comSystem.dynamics->setSystemMatrices(Eigen::Matrix2d::Zero(), Eigen::Matrix2d::Identity());
    m_comSystem.dynamics->setState(
        {iDynTree::toEigen(m_kinDynWithDesired.kindyn->getCenterOfMassPosition()).head<2>()});

    m_comSystem.integrator = std::make_shared<ForwardEuler<LinearTimeInvariantSystem>>();
    m_comSystem.integrator->setIntegrationStep(m_dT);
    m_comSystem.integrator->setDynamicalSystem(m_comSystem.dynamics);

    m_myfile.open("example.txt");
    m_myfile << "torso_pitch_des, torso_roll_des, torso_yaw_des, l_shoulder_pitch_des, "
              "l_shoulder_roll_des, l_shoulder_yaw_des, l_elbow_des, r_shoulder_pitch_des, "
              "r_shoulder_roll_des, r_shoulder_yaw_des, r_elbow_des, l_hip_pitch_des, "
              "l_hip_roll_des, l_hip_yaw_des, l_knee_des, l_ankle_pitch_des, l_ankle_roll_des, "
              "r_hip_pitch_des, r_hip_roll_des, r_hip_yaw_des, r_knee_des, r_ankle_pitch_des, "
              "r_ankle_roll_des, "
              "torso_pitch, torso_roll, torso_yaw, l_shoulder_pitch, "
              "l_shoulder_roll, l_shoulder_yaw, l_elbow, r_shoulder_pitch, "
              "r_shoulder_roll, r_shoulder_yaw, r_elbow, l_hip_pitch, "
              "l_hip_roll, l_hip_yaw, l_knee, l_ankle_pitch, l_ankle_roll, "
              "r_hip_pitch, r_hip_roll, r_hip_yaw, r_knee, r_ankle_pitch, "
              "r_ankle_roll, "
              "base_x, base_y, base_z, com_x, com_y, com_z, des_com_x, "
              "des_com_y, des_com_z, "
              "lf_des_x, lf_des_y, lf_des_z, lf_meas_x, lf_meas_y, lf_meas_z, lf_meas_d_x, "
              "lf_meas_d_y, lf_meas_d_z, rf_des_x, rf_des_y, rf_des_z, rf_meas_x, rf_meas_y, "
              "rf_meas_z, rf_meas_d_x, rf_meas_d_y, rf_meas_d_z , zmp_des_x, zmp_des_y, "
              "zmp_meas_x, zmp_meas_y, dcom_x, dcom_y, dcom_z, des_dcom_x, des_dcom_y, des_dcom_z,"
              "d_torso_pitch_des, d_torso_roll_des, d_torso_yaw_des, d_l_shoulder_pitch_des, "
              "d_l_shoulder_roll_des, d_l_shoulder_yaw_des, d_l_elbow_des, d_r_shoulder_pitch_des, "
              "d_r_shoulder_roll_des, d_r_shoulder_yaw_des, d_r_elbow_des, d_l_hip_pitch_des, "
              "d_l_hip_roll_des, d_l_hip_yaw_des, d_l_knee_des, d_l_ankle_pitch_des, "
              "d_l_ankle_roll_des, "
              "d_r_hip_pitch_des, d_r_hip_roll_des, d_r_hip_yaw_des, d_r_knee_des, "
              "d_r_ankle_pitch_des, "
              "d_r_ankle_roll_des, "
              "d_torso_pitch, d_torso_roll, d_torso_yaw, d_l_shoulder_pitch, "
              "d_l_shoulder_roll, d_l_shoulder_yaw, d_l_elbow, d_r_shoulder_pitch, "
              "d_r_shoulder_roll, d_r_shoulder_yaw, d_r_elbow, d_l_hip_pitch, "
              "d_l_hip_roll, d_l_hip_yaw, d_l_knee, d_l_ankle_pitch, d_l_ankle_roll, "
              "d_r_hip_pitch, d_r_hip_roll, d_r_hip_yaw, d_r_knee, d_r_ankle_pitch, "
              "d_r_ankle_roll, rf_adj_x, rf_adj_y, rf_adj_z, lf_adj_x, lf_adj_y, lf_adj_z, "
              "external_fx, external_fy, external_fz, external_taux, external_tauy, external_tauz, "
              "computational_time, "
              "ang_mom_x, ang_mom_y, ang_mom_z";

    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            m_myfile << " , contact_" << i << "_corner_" << j << "_x";
            m_myfile << " , contact_" << i << "_corner_" << j << "_y";
            m_myfile << " , contact_" << i << "_corner_" << j << "_z";
        }
    }

    m_myfile << std::endl;

    BipedalLocomotion::log()->info("{} The WholeBodyQPBlock has been configured.", logPrefix);

    return true;
}

const WholeBodyQPBlock::Output& WholeBodyQPBlock::getOutput() const
{
    return m_output;
}

bool WholeBodyQPBlock::setInput(const Input& input)
{
    m_input = input;
    return true;
}

bool WholeBodyQPBlock::evaluateZMP(Eigen::Ref<Eigen::Vector2d> zmp)
{
    using namespace BipedalLocomotion;
    Eigen::Vector3d zmpRight, zmpLeft;
    zmpLeft.setZero();
    zmpRight.setZero();
    double totalZ = 0;

    Math::Wrenchd leftWrench = Math::Wrenchd::Zero();
    for (const auto& [key, value] : m_leftFootContacWrenches)
    {
        leftWrench += value.wrench;
    }
    Math::Wrenchd rightWrench = Math::Wrenchd::Zero();
    for (const auto& [key, value] : m_rightFootContacWrenches)
    {
        rightWrench += value.wrench;
    }

    double zmpLeftDefined = 0.0, zmpRightDefined = 0.0;
    if (rightWrench.force()(2) < 0.001)
        zmpRightDefined = 0.0;
    else
    {
        zmpRight(0) = -rightWrench.torque()(1) / rightWrench.force()(2);
        zmpRight(1) = rightWrench.torque()(0) / rightWrench.force()(2);
        zmpRight(2) = 0.0;
        zmpRightDefined = 1.0;
        totalZ += rightWrench.force()(2);
    }

    if (leftWrench.force()(2) < 0.001)
        zmpLeftDefined = 0.0;
    else
    {
        zmpLeft(0) = -leftWrench.torque()(1) / leftWrench.force()(2);
        zmpLeft(1) = leftWrench.torque()(0) / leftWrench.force()(2);
        zmpLeft(2) = 0.0;
        zmpLeftDefined = 1.0;
        totalZ += leftWrench.force()(2);
    }

    if (totalZ < 0.1)
    {
        BipedalLocomotion::log()->error("[WholeBodyQPBlock::evaluateZMP] The total z-component of "
                                        "contact wrenches is too low.");
        return false;
    }

    manif::SE3d I_H_lf = BipedalLocomotion::Conversions::toManifPose(
        m_kinDynWithMeasured.kindyn->getWorldTransform("l_sole"));
    manif::SE3d I_H_rf = BipedalLocomotion::Conversions::toManifPose(
        m_kinDynWithMeasured.kindyn->getWorldTransform("r_sole"));

    zmpLeft = I_H_lf.act(zmpLeft);
    zmpRight = I_H_rf.act(zmpRight);

    // the global zmp is given by a weighted average
    zmp = ((leftWrench.force()(2) * zmpLeftDefined) / totalZ) * zmpLeft.head<2>()
          + ((rightWrench.force()(2) * zmpRightDefined) / totalZ) * zmpRight.head<2>();

    return true;
}

Eigen::Vector2d WholeBodyQPBlock::computeDesiredZMP(
    const std::map<std::string, BipedalLocomotion::Contacts::ContactWithCorners>& contacts)
{
    Eigen::Vector2d zmp = Eigen::Vector2d::Zero();
    double totalZ = 0;
    Eigen::Vector3d localZMP;

    for (const auto& [key, contact] : contacts)
    {
        BipedalLocomotion::Math::Wrenchd totalWrench = BipedalLocomotion::Math::Wrenchd::Zero();
        for (const auto& corner : contact.corners)
        {
            totalWrench.force() = corner.force;
            totalWrench.torque() += corner.position.cross(contact.pose.asSO3().act(corner.force));
        }
        if (totalWrench.force()(2) > 0.001)
        {
            totalZ += totalWrench.force()(2);
            localZMP(0) = -totalWrench.torque()(1) / totalWrench.force()(2);
            localZMP(1) = totalWrench.torque()(0) / totalWrench.force()(2);
            localZMP(2) = 0.0;

            // the wrench is already expressed in mixed we have just to translate it
            // if left
            if (key == "left_foot")
            {
                manif::SE3d temptransofrm = BipedalLocomotion::Conversions::toManifPose(
                    m_kinDynWithDesired.kindyn->getWorldTransform("l_sole"));
                zmp += totalWrench.force()(2) * temptransofrm.act(localZMP).head<2>();
            }
            // if left
            else if (key == "right_foot")
            {
                manif::SE3d temptransofrm = BipedalLocomotion::Conversions::toManifPose(
                    m_kinDynWithDesired.kindyn->getWorldTransform("r_sole"));
                zmp += totalWrench.force()(2) * temptransofrm.act(localZMP).head<2>();
            }
            else
            {
                BipedalLocomotion::log()->error("Problem in evaluated the desired zmp");
            }
        }
    }

    zmp = zmp / totalZ;

    return zmp;
}

bool WholeBodyQPBlock::advance()
{
    bool shouldAdvance = false;

    constexpr auto errorPrefix = "[WholeBodyQPBlock::advance]";

    Eigen::Vector2d desiredZMP = Eigen::Vector2d::Zero();
    Eigen::Vector2d measuredZMP = Eigen::Vector2d::Zero();

    if (!m_sensorBridge.advance())
    {
        BipedalLocomotion::log()->error("{} Unable to get the robot state.", errorPrefix);
        return false;
    }

    m_sensorBridge.getJointPositions(m_currentJointPos);
    m_sensorBridge.getJointVelocities(m_currentJointVel);

    Eigen::Vector3d gravity;
    gravity.setZero();
    gravity(2) = -BipedalLocomotion::Math::StandardAccelerationOfGravitation;

    if (!this->updateFloatingBase())
    {
        BipedalLocomotion::log()->error("{} Unable to update the floating base.", errorPrefix);
        return false;
    }

    m_baseTransform = m_floatingBaseEstimator.getOutput().basePose;
    m_baseVelocity = m_floatingBaseEstimator.getOutput().baseTwist;

    m_kinDynWithMeasured.kindyn->setRobotState(m_baseTransform.transform(),
                                               m_currentJointPos,
                                               iDynTree::make_span(m_baseVelocity.data(),
                                                                   manif::SE3d::Tangent::DoF),
                                               m_currentJointVel,
                                               gravity);

    /////// update kinDyn
    m_kinDynWithDesired.kindyn->setRobotState(m_baseTransform.transform(),
                                              m_desJointPos,
                                              iDynTree::make_span(m_baseVelocity.data(),
                                                                  manif::SE3d::Tangent::DoF),
                                              m_desJointVel,
                                              gravity);

    for (auto& [key, value] : m_leftFootContacWrenches)
    {
        if (!m_sensorBridge.getCartesianWrench(key, value.wrench))
        {
            BipedalLocomotion::log()->error("{} Unable to get the left wrench named {}.",
                                            errorPrefix,
                                            key);
            return false;
        }
    }

    for (auto& [key, value] : m_rightFootContacWrenches)
    {
        if (!m_sensorBridge.getCartesianWrench(key, value.wrench))
        {
            BipedalLocomotion::log()->error("{} Unable to get the left wrench named {}.",
                                            errorPrefix,
                                            key);
            return false;
        }
    }

    BipedalLocomotion::Math::Wrenchd totalExternalWrench = BipedalLocomotion::Math::Wrenchd::Zero();
    for (auto& [key, value] : m_externalContactWrenches)
    {
        if (!m_sensorBridge.getCartesianWrench(key, value.wrench))
        {
            BipedalLocomotion::log()->error("{} Unable to get the left wrench named {}.",
                                            errorPrefix,
                                            key);
            return false;
        }

        manif::SO3d rotation = BipedalLocomotion::Conversions::toManifRot(
            m_kinDynWithMeasured.kindyn->getWorldTransform("root_link").getRotation());
        totalExternalWrench += rotation * value.wrench;
    }

    if (totalExternalWrench.force().norm() < 30)
    {
        totalExternalWrench.setZero();
    } else
    {
        totalExternalWrench.force()(2) = 0;

        const double totalMass = 52.0102;
        totalExternalWrench = totalExternalWrench / totalMass;
    }

    if (m_input.contacts.size() != 0)
    {
        Eigen::Vector3d force = Eigen::Vector3d::Zero();
        for (const auto& [key, value] : m_input.contacts)
        {
            for (const auto& c : value.corners)
            {
                force += c.force;
            }
        }

        if (force.norm() > 0.01)
        {
            m_centroidalSystem.dynamics->setControlInput({m_input.contacts, m_input.externalWrench});
            m_centroidalSystem.integrator->integrate(0, m_dT);
            desiredZMP = this->computeDesiredZMP(m_input.contacts);
            updateContactPhaseList(m_input.nextPlannedContact, m_contactListMap);

            shouldAdvance = true;

            auto newPhaseIt = m_contactPhaselist.getPresentPhase(m_absoluteTime);
            if(newPhaseIt != m_phaseIt)
            {
                // check if new contact is established
                if (m_phaseIt->activeContacts.size() == 1 && newPhaseIt->activeContacts.size() == 2)
                {
                    // update the contact phase list
                    m_contactPhaselist.setLists(m_contactListMap);
                    m_leftFootPlanner.setContactListWithotResettingTheInternalTime(
                        m_contactPhaselist.lists().at("left_foot"));
                    m_rightFootPlanner.setContactListWithotResettingTheInternalTime(
                        m_contactPhaselist.lists().at("right_foot"));
                    m_fixedFootDetector.setContactPhaseListWithoutResetInternalTime(
                        m_contactPhaselist);

                    // the iterators have been modified we have to compute the new one
                    m_phaseIt = m_contactPhaselist.getPresentPhase(m_absoluteTime);
                } else
                {
                    // the iterators did not change no need to get the present phase again
                    m_phaseIt = newPhaseIt;
                }
            }
            newPhaseIt = m_contactPhaselist.getPresentPhase(m_absoluteTime);

            if (newPhaseIt->activeContacts.size() == 2)
            {
                // if the left foot is in contact the adjusted cannot be used. If next planned
                // right is found this will be avaiable till the next contact enabled
                m_isLeftAsjustedReady = false;
                m_isLeftAdjusterAlreadyUsed = false;
            }
            // for each contact we check if is currently enabled or not
            else
            {
                // is the next planned contact available
                const auto nextPlannedLeft = m_input.nextPlannedContact.find("left_foot");
                if (nextPlannedLeft != m_input.nextPlannedContact.end())
                {
                    m_isLeftAsjustedReady = true;
                    if (nextPlannedLeft->second.pose.translation().head<2>()
                        != m_nextLeftFootPositionXY)
                    {
                        m_nextLeftFootPositionXY
                            = nextPlannedLeft->second.pose.translation().head<2>();

                        if (m_isLeftAdjusterAlreadyUsed)
                        {
                            if (m_absoluteTime > nextPlannedLeft->second.activationTime)
                            {
                                BipedalLocomotion::log()
                                    ->error("{} Left foot adsjustement. The left absolute time is "
                                            "higher that the activation time. abs {} activation{}",
                                            errorPrefix,
                                            m_absoluteTime,
                                            nextPlannedLeft->second.activationTime);
                                return false;
                            }
                            auto state = m_leftFootAdjuster.getOutput();
                            m_leftFootAdjuster.setInitialConditions(state.velocity,
                                                                    state.acceleration);
                            m_leftFootAdjuster.setKnots({state.position, m_nextLeftFootPositionXY},
                                                        {m_absoluteTime,
                                                         nextPlannedLeft->second.activationTime});
                        } else
                        {
                            if (m_absoluteTime > nextPlannedLeft->second.activationTime)
                            {
                                BipedalLocomotion::log()
                                    ->error("{} Left foot adsjustement. The left absolute time is "
                                            "higher that the activation time. abs {} activation{}",
                                            errorPrefix,
                                            m_absoluteTime,
                                            nextPlannedLeft->second.activationTime);
                                return false;
                            }

                            m_leftFootAdjuster.setInitialConditions(m_leftFootPlanner.getOutput()
                                                                        .mixedVelocity.coeffs()
                                                                        .head<2>(),
                                                                    m_leftFootPlanner.getOutput()
                                                                        .mixedAcceleration.coeffs()
                                                                        .head<2>());
                            m_leftFootAdjuster.setKnots({m_leftFootPlanner.getOutput()
                                                             .transform.translation()
                                                             .head<2>(),
                                                         m_nextLeftFootPositionXY},
                                                        {m_absoluteTime,
                                                         nextPlannedLeft->second.activationTime});
                        }
                        m_isLeftAdjusterAlreadyUsed = true;
                    }
                }
            }

            if (newPhaseIt->activeContacts.size() == 2)
            {
                // if the right foot is in contact the adjusted cannot be used. If next planned
                // right is found this will be avaiable till the next contact enabled
                m_isRightAsjustedReady = false;
                m_isRightAdjusterAlreadyUsed = false;
            }
            // for each contact we check if is currently enabled or not
            else
            {
                // is the next planned contact available
                const auto nextPlannedRight = m_input.nextPlannedContact.find("right_foot");
                if (nextPlannedRight != m_input.nextPlannedContact.end())
                {
                    m_isRightAsjustedReady = true;
                    if (nextPlannedRight->second.pose.translation().head<2>()
                        != m_nextRightFootPositionXY)
                    {
                        m_nextRightFootPositionXY
                            = nextPlannedRight->second.pose.translation().head<2>();

                        if (m_isRightAdjusterAlreadyUsed)
                        {
                            if (m_absoluteTime > nextPlannedRight->second.activationTime)
                            {
                                BipedalLocomotion::log()
                                    ->error("{} Left foot adsjustement. The left absolute time is "
                                            "higher that the activation time. abs {} activation{}",
                                            errorPrefix,
                                            m_absoluteTime,
                                            nextPlannedRight->second.activationTime);
                                return false;
                            }

                            auto state = m_rightFootAdjuster.getOutput();
                            m_rightFootAdjuster.setInitialConditions(state.velocity,
                                                                     state.acceleration);
                            m_rightFootAdjuster.setKnots({state.position,
                                                          m_nextRightFootPositionXY},
                                                         {m_absoluteTime,
                                                          nextPlannedRight->second.activationTime});
                        } else
                        {

                            if (m_absoluteTime > nextPlannedRight->second.activationTime)
                            {
                                BipedalLocomotion::log()
                                    ->error("{} Left foot adsjustement. The left absolute time is "
                                            "higher that the activation time. abs {} activation{}",
                                            errorPrefix,
                                            m_absoluteTime,
                                            nextPlannedRight->second.activationTime);
                                return false;
                            }

                            m_rightFootAdjuster.setInitialConditions(m_rightFootPlanner.getOutput()
                                                                         .mixedVelocity.coeffs()
                                                                         .head<2>(),
                                                                     m_rightFootPlanner.getOutput()
                                                                         .mixedAcceleration.coeffs()
                                                                         .head<2>());
                            m_rightFootAdjuster.setKnots({m_rightFootPlanner.getOutput()
                                                              .transform.translation()
                                                              .head<2>(),
                                                          m_nextRightFootPositionXY},
                                                         {m_absoluteTime,
                                                          nextPlannedRight->second.activationTime});
                        }
                        m_isRightAdjusterAlreadyUsed = true;
                    }
                }
            }
        }
    }

    if (!this->evaluateZMP(measuredZMP))
    {
        BipedalLocomotion::log()->error("{} Unable to evaluate the measured zmp.", errorPrefix);
        return false;
    }

    // if (!m_leftFootPlanner.getOutput().mixedVelocity.coeffs().isZero())
    // {
    //     m_IKandTasks.leftFootTask->enableControl();
    // } else
    // {
    //     m_IKandTasks.leftFootTask->disableControl();
    // }

    // if (!m_rightFootPlanner.getOutput().mixedVelocity.coeffs().isZero())
    // {
    //     m_IKandTasks.rightFootTask->enableControl();
    // } else
    // {
    //     m_IKandTasks.rightFootTask->disableControl();
    // }

    manif::SE3d transformLeft = m_leftFootPlanner.getOutput().transform;
    if (!m_isLeftAsjustedReady)
    {
        m_IKandTasks.leftFootTask->setSetPoint(m_leftFootPlanner.getOutput().transform,
                                               m_leftFootPlanner.getOutput().mixedVelocity);

    } else
    {

        Eigen::Vector3d newPosition = transformLeft.translation();
        newPosition.head<2>() = m_leftFootAdjuster.getOutput().position;
        transformLeft.translation(newPosition);
        manif::SE3d::Tangent velocity = m_leftFootPlanner.getOutput().mixedVelocity;
        velocity.lin().head<2>() = m_leftFootAdjuster.getOutput().velocity;

        m_IKandTasks.leftFootTask->setSetPoint(transformLeft, velocity);
    }

    manif::SE3d transform = m_rightFootPlanner.getOutput().transform;
    manif::SE3d::Tangent velocity = m_rightFootPlanner.getOutput().mixedVelocity;
    if (m_isRightAsjustedReady)
    {
        Eigen::Vector3d newPosition = transform.translation();
        newPosition.head<2>() = m_rightFootAdjuster.getOutput().position;
        transform.translation(newPosition);

        velocity.lin().head<2>() = m_rightFootAdjuster.getOutput().velocity;
    }

    m_IKandTasks.rightFootTask->setSetPoint(transform, velocity);

    Eigen::Vector3d comdes = std::get<0>(m_centroidalSystem.dynamics->getState());
    // comdes(2) = m_com0z;

    Eigen::Vector3d dcomdes = std::get<1>(m_centroidalSystem.dynamics->getState());
    // dcomdes(2) = 0;

    if (shouldAdvance)
    {
        m_CoMZMPController.setSetPoint(dcomdes.head<2>(), comdes.head<2>(), desiredZMP);
        m_CoMZMPController.setFeedback(iDynTree::toEigen(
                                           m_kinDynWithMeasured.kindyn->getCenterOfMassPosition())
                                           .head<2>(),
                                       measuredZMP,
                                       0);
        m_CoMZMPController.advance();
        dcomdes.head<2>() = m_CoMZMPController.getOutput();
        m_comSystem.dynamics->setControlInput({dcomdes.head<2>()});
        m_comSystem.integrator->integrate(0, m_dT);
        comdes.head<2>() = std::get<0>(m_comSystem.integrator->getSolution());
    }

    m_IKandTasks.comTask->setSetPoint(comdes, dcomdes);

    // const double yawLeft
    //     = m_leftFootPlanner.getOutput().transform.quat().toRotationMatrix().eulerAngles(2, 1,
    //     0)(0);

    // const double yawRight
    //     = m_rightFootPlanner.getOutput().transform.quat().toRotationMatrix().eulerAngles(2, 1,
    //     0)(
    //         0);

    // const double meanYaw = std::atan2(std::sin(yawLeft) + std::sin(yawRight),
    //                                   std::cos(yawLeft) + std::cos(yawRight));

    // // manif::SO3d chestOrientation
    // //     = manif::SO3d(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())
    // //                   * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ())
    // //                   * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()));

    // manif::SO3d chestOrientation
    //     = manif::SO3d(Eigen::AngleAxisd(meanYaw, Eigen::Vector3d::UnitZ()));

    // ((0.0 0.0 1.0),
    //  (1.0 0.0 0.0),
    //  (0.0 1.0 0.0));

    m_IKandTasks.chestTask->setSetPoint(manif::SO3d::Identity(), manif::SO3d::Tangent::Zero());

    if (!m_IKandTasks.ik->advance())
    {
        BipedalLocomotion::log()->error("{} Unable to solve the IK problem", errorPrefix);
        return false;
    }

    if (shouldAdvance)
    {
        // advance the fixed foot class
        m_fixedFootDetector.advance();
        m_leftFootPlanner.advance();
        m_rightFootPlanner.advance();

        // if left adjuster can be used please advance
        if (m_isLeftAsjustedReady)
        {
            m_leftFootAdjuster.advance();
        }
        if (m_isRightAsjustedReady)
        {
            m_rightFootAdjuster.advance();
        }


        m_absoluteTime += m_dT;
        m_absoluteTime = roundoff(m_absoluteTime, 3);
        if (m_absoluteTime > 40 /1.5)
        {
            BipedalLocomotion::log()->info("{} Experiment ended", errorPrefix);
            return false;
        }
    }

    m_system.dynamics->setControlInput({m_IKandTasks.ik->getOutput().baseVelocity.coeffs(),
                                        m_IKandTasks.ik->getOutput().jointVelocity});
    m_system.integrator->integrate(0, m_dT);

    const auto& [basePosition, baseRotation, jointPosition] = m_system.integrator->getSolution();

    m_desJointPos = jointPosition;
    m_desJointVel = m_IKandTasks.ik->getOutput().jointVelocity;

    // m_kinDynWithMeasured.kindyn->getCenterOfMassPosition(m_output.com);
    // m_kinDynWithMeasured.kindyn->getCenterOfMassVelocity(m_output.dcom);
    // m_output.angularMomentum
    //     = iDynTree::toEigen(
    //         m_kinDynWithMeasured.kindyn->getCentroidalTotalMomentum().getAngularVec3());

    m_output.leftFoot = BipedalLocomotion::Conversions::toManifPose(
        m_kinDynWithMeasured.kindyn->getWorldTransform("l_sole"));

    m_output.rightFoot = BipedalLocomotion::Conversions::toManifPose(
        m_kinDynWithMeasured.kindyn->getWorldTransform("r_sole"));

    // TODO remove me
    m_output.com = std::get<0>(m_centroidalSystem.dynamics->getState());
    m_output.dcom = std::get<1>(m_centroidalSystem.dynamics->getState());
    m_output.angularMomentum = std::get<2>(m_centroidalSystem.dynamics->getState());
    // m_output.com = iDynTree::toEigen(m_kinDynWithMeasured.kindyn->getCenterOfMassPosition());
    // m_output.dcom = iDynTree::toEigen(m_kinDynWithMeasured.kindyn->getCenterOfMassVelocity());
    // m_output.angularMomentum = std::get<2>(m_centroidalSystem.dynamics->getState());
    m_output.totalExternalWrench = totalExternalWrench;

    m_output.isValid = true;

    if(shouldAdvance)
    {
    Eigen::IOFormat
        CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "", "");
    m_myfile
        << m_desJointPos.format(CommaInitFmt) << ", " << m_currentJointPos.format(CommaInitFmt)
        << ", " << m_baseTransform.translation().format(CommaInitFmt) << ", "
        << iDynTree::toEigen(m_kinDynWithMeasured.kindyn->getCenterOfMassPosition())
               .format(CommaInitFmt)
        << ", " << std::get<0>(m_centroidalSystem.dynamics->getState()).format(CommaInitFmt) << ", "
        << m_leftFootPlanner.getOutput().transform.translation().format(CommaInitFmt) << ", "
        << iDynTree::toEigen(m_kinDynWithMeasured.kindyn->getWorldTransform("l_sole").getPosition())
               .format(CommaInitFmt)
        << ", "
        << iDynTree::toEigen(m_kinDynWithDesired.kindyn->getWorldTransform("l_sole").getPosition())
               .format(CommaInitFmt)
        << "," << m_rightFootPlanner.getOutput().transform.translation().format(CommaInitFmt)
        << ", "
        << iDynTree::toEigen(m_kinDynWithMeasured.kindyn->getWorldTransform("r_sole").getPosition())
               .format(CommaInitFmt)
        << ", "
        << iDynTree::toEigen(m_kinDynWithDesired.kindyn->getWorldTransform("r_sole").getPosition())
               .format(CommaInitFmt)
        << ", " << desiredZMP.format(CommaInitFmt) << ", " << measuredZMP.format(CommaInitFmt)
        << ", "
        << iDynTree::toEigen(m_kinDynWithMeasured.kindyn->getCenterOfMassVelocity())
               .format(CommaInitFmt)
        << ", " << std::get<1>(m_centroidalSystem.dynamics->getState()).format(CommaInitFmt) << ", "
        << m_desJointVel.format(CommaInitFmt) << ", " << m_currentJointVel.format(CommaInitFmt)
        << ", " << transform.translation().format(CommaInitFmt) << ", "
        << transformLeft.translation().format(CommaInitFmt) << ", "
        << totalExternalWrench.transpose().format(CommaInitFmt) << ", " << m_input.computationalTime
        << ", " << m_output.angularMomentum.format(CommaInitFmt);

    for (const auto& [key, contact] : m_input.contacts)
    {
        for (const auto& corner : contact.corners)
        {
          m_myfile << ", " << corner.force.transpose().format(CommaInitFmt);
        }
    }
    m_myfile << std::endl;

    }

    if (!m_robotControl.setReferences(jointPosition,
                                      BipedalLocomotion::RobotInterface::IRobotControl::
                                          ControlMode::PositionDirect))
    {
        BipedalLocomotion::log()->error("{} Unable to set the reference", errorPrefix);
        return false;
    }

    return true;
}

bool WholeBodyQPBlock::isOutputValid() const
{
    return true;
}

bool WholeBodyQPBlock::close()
{
    m_myfile.close();
    return true;
}
