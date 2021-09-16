<h1 align="center">
 Centroidal MPC for walking
</h1>


### Installation

The software stored in this repository depends on an experimental version of the [`bipedal-locomotion-framework`](https://github.com/ami-iit/bipedal-locomotion-framework/tree/feature/CentroidalMPC)  library.

You can easily all the dependencies of `bipedal-locomotion-framework` with [`conda`](https://docs.conda.io) using the following command

```
conda install cmake compilers make ninja pkg-config
conda install -c conda-forge -c robotology idyntree yarp libmatio matio-cpp lie-group-controllers eigen qhull "casadi>=3.5.5" cppad spdlog catch2 nlohmann_json manif
```

Once the dependencies has been installed you can compile `bipedal-locomotion-framework` as follow

```
git clone https://github.com/ami-iit/bipedal-locomotion-framework.git
cd bipedal-locomotion-framework
git checkout e00ccfde545a0d94d986d0159066b98d5820dbe4
mkdir build
cd build
cmake -DCMAKE_PREFIX_PATH=<path/where/you/want/to/install/blf>
make install
```

You can finally clone this repository and build it using the usual cmake machinery

### Running

You can run a simulation scenario in gazebo. Please install `gazebo` and the associated plugins and the robot models with

```
conda install -c conda-forge -c robotology gazebo-yarp-plugins icub-models
```

Then you should set the `YARP_ROBOT_NAME` environment variable to `iCubGazeboV3`

You can spawn the world with

```
gazebo -slibgazebo_yarp_clock.so centroidal_mpc_icub3/world
```

Please run the simulation with the following command

```
YARP_CLOCK=/clock cmw-walking
```

### Maintainer

This repository is maintained by:

|                                                              |                                                      |
| :----------------------------------------------------------: | :--------------------------------------------------: |
| [<img src="https://github.com/GiulioRomualdi.png" width="40">](https://github.com/GiulioRomualdi) | [@GiulioRomualdi](https://github.com/GiulioRomualdi) |
