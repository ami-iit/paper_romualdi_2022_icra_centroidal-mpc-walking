<h1 align="center">
 Online Non-linear Centroidal MPC for Humanoid Robot Locomotion with Step Adjustment
</h1>

<div align="center">

G. Romualdi, S. Dafarra, G. L'Erario, I. Sorrentino, S. Traversaro and D. Pucci "Online Non-linear Centroidal MPC for Humanoid Robot Locomotion with Step Adjustment" in IEEE 2022 International Conference on Robotics and Automation (ICRA)

</div>

<p align="center">


https://user-images.githubusercontent.com/16744101/163016195-2151ae19-6a9c-4fb5-9a57-92ee2cd8c899.mp4


<div align="center">
2022 International Conference on Robotics and Automation (ICRA)
</div>
 
<div align="center">
  <a href="#reproducing-the-experiments"><b>Installation</b></a> |
  <a href="https://arxiv.org/abs/2203.04489"><b>arXiv</b></a> |
  <a href="https://www.youtube.com/watch?v=u7vCgE2w_vY9"><b>YouTube</b></a>
</div>


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

🎥 Please check [this video](https://user-images.githubusercontent.com/16744101/134700608-33d8fcb2-931d-4ffd-8355-59650386dd66.mp4) if you want to watch a simulation experiment.

⚠️ **Known issue**: The `gazebo` real-time factor is scaled of factor 10. This is necessary since the linear solver used by `IPOPT` in the docker image is `mumps`. Unfortunately, other linear solvers (e.g. `ma27`) can be downloaded but not redistributed. Please check [here](https://www.hsl.rl.ac.uk/ipopt/).
If you want to speed up the simulation you may install `IPOPT 3.13.4` with `CoinBrew` +  `HSL solver` as explained [here](https://gist.github.com/GiulioRomualdi/22fddb949e7b09bb53ca2ff72cbf8cb6).

## Citing this work

If you find the work useful, please consider citing:

```bibtex
@article{https://doi.org/10.48550/arxiv.2203.04489,
  doi = {10.48550/ARXIV.2203.04489},
  url = {https://arxiv.org/abs/2203.04489},
  author = {Romualdi, Giulio and Dafarra, Stefano and L'Erario, Giuseppe and Sorrentino, Ines and Traversaro, Silvio and Pucci, Daniele},
  keywords = {Robotics (cs.RO), FOS: Computer and information sciences, FOS: Computer and information sciences},
  title = {Online Non-linear Centroidal MPC for Humanoid Robot Locomotion with Step Adjustment},
  publisher = {arXiv},
  year = {2022},
  copyright = {arXiv.org perpetual, non-exclusive license}
}
```



## Maintainer

This repository is maintained by:

|                                                              |                                                      |
| :----------------------------------------------------------: | :--------------------------------------------------: |
| [<img src="https://github.com/GiulioRomualdi.png" width="40">](https://github.com/GiulioRomualdi) | [@GiulioRomualdi](https://github.com/GiulioRomualdi) |
