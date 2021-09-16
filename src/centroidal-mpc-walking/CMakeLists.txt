# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.
# Authors: Giulio Romualdi

add_centroidal_mpc_walking_application(
  NAME            walking
  SOURCES         src/Main.cpp src/WholeBodyQPBlock.cpp src/CentroidalMPCBlock.cpp
  HEADERS         include/CentroidalMPCWalking/WholeBodyQPBlock.h include/CentroidalMPCWalking/CentroidalMPCBlock.h
  LINK_LIBRARIES  BipedalLocomotion::ParametersHandlerYarpImplementation
                  BipedalLocomotion::RobotInterfaceYarpImplementation
                  BipedalLocomotion::FloatingBaseEstimators
                  BipedalLocomotion::IK
                  BipedalLocomotion::ContinuousDynamicalSystem
                  BipedalLocomotion::System
                  BipedalLocomotion::SystemYarpImplementation
                  BipedalLocomotion::ReducedModelControllers
                  BipedalLocomotion::Planners
                  BipedalLocomotion::ManifConversions
                  BipedalLocomotion::ContactDetectors
                  BipedalLocomotion::SimplifiedModelControllers
  )

install_ini_files(${CMAKE_CURRENT_SOURCE_DIR}/config)

