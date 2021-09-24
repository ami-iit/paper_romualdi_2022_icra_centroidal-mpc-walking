<h1 align="center">
 Centroidal MPC for walking
</h1>

## Short description of the repository
This repository contains the code that implements a non-linear online MPC for bipedal locomotion that considers the Centroidal dynamics of the floating base system.

## Reproducing the experiments

We support running the experiments via the provided Docker image.

1. Pull the docker image:
    ```console
    docker pull ghcr.io/ami-iit/centroidal-mpc-walking-docker:latest
    ```
2. Launch the container:
    ```console
    xhost +
    docker run -it --rm  \
               --device=/dev/dri:/dev/dri \
               --user user \
               --env="DISPLAY=$DISPLAY"  \
               --net=host \
               ghcr.io/ami-iit/centroidal-mpc-walking-docker:latest
    ```
3. Wait for `gazebo` running and then start the experiment


⚠️ **Known issue**: The `gazebo` real-time factor is scaled of factor 10. This is necessary since the linear solver used by `IPOPT` in the docker image is `mumps`. Unfortunately, other linear solvers (e.g. `ma27`) can be downloaded but not redistributed. Please check [here](https://www.hsl.rl.ac.uk/ipopt/).
If you want to speed up the simulation you may install `IPOPT 3.13.4` with `CoinBrew` +  `HSL solver` as explained [here](https://gist.github.com/GiulioRomualdi/22fddb949e7b09bb53ca2ff72cbf8cb6).

### Maintainer

This repository is maintained by:

|                                                              |                                                      |
| :----------------------------------------------------------: | :--------------------------------------------------: |
| [<img src="https://github.com/GiulioRomualdi.png" width="40">](https://github.com/GiulioRomualdi) | [@GiulioRomualdi](https://github.com/GiulioRomualdi) |
