FROM condaforge/mambaforge:4.9.2-5 as conda
LABEL org.opencontainers.image.title="CentroidalMPC Docker Image"
LABEL org.opencontainers.image.description="Infrastructure for reproducing CentroidalMPC experiments"
LABEL org.opencontainers.image.source="https://raw.githubusercontent.com/ami-iit/centroidal-mpc/master/dockerfiles/Dockerfile"
LABEL org.opencontainers.image.authors="Giulio Romualdi <giulio.romualdi@iit.it>"


SHELL ["/bin/bash", "-c"]

# Non-interactive installation mode
ENV DEBIAN_FRONTEND=noninteractive

# Update apt database
RUN apt update

# Set the locale
RUN apt install -y -qq apt-utils locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Install essentials
RUN apt install -y -qq build-essential ca-certificates cmake cmake-curses-gui curl emacs-nox git glmark2 gnupg2 htop iputils-ping lsb-release mesa-utils nano psmisc sudo unzip vim wget libqt5opengl5-dev && \
   rm -rf /var/lib/apt/lists/*


COPY environment.yml .
RUN conda env update --file environment.yml

# install bipedal locomotion framework
RUN git clone https://github.com/ami-iit/bipedal-locomotion-framework.git && \
    cd bipedal-locomotion-framework && \
    git checkout 6cfe29fcfdd5c598bb0baa809b91c96139e4bcf1 && \
    mkdir build && \
    cd build && \
    cmake -GNinja -DCMAKE_INSTALL_PREFIX=/opt/conda .. && \
    cmake --build . --target install


# install icub-models
COPY icub-models.patch .

RUN git clone https://github.com/robotology/icub-models.git && \
    cd icub-models && \
    git checkout v1.21.1 && \
    cp ../icub-models.patch . && \
    git apply ./icub-models.patch && \
    mkdir build && \
    cd build && \
    cmake -GNinja -DCMAKE_INSTALL_PREFIX=/opt/conda .. && \
    cmake --build . --target install


# install the centroidal mpc
RUN git clone https://github.com/ami-iit/centroidal-mpc-walking.git && \
    cd centroidal-mpc-walking && \
    mkdir build && \
    cd build && \
    cmake -GNinja -DCMAKE_INSTALL_PREFIX=/opt/conda .. && \
    cmake --build . --target install


# Create user with passwordless sudo
RUN useradd -l -G sudo -md /home/user -s /bin/bash -p user user
RUN sed -i.bkp -e 's/%sudo\s\+ALL=(ALL\(:ALL\)\?)\s\+ALL/%sudo ALL=NOPASSWD:ALL/g' /etc/sudoers

# Switch to user
USER user


# Gazebo related env variables (see http://gazebosim.org/tutorials?tut=components#EnvironmentVariables )
RUN echo 'export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH:+${GAZEBO_PLUGIN_PATH}:}/opt/conda/lib' >> ~/.bashrc
RUN echo 'export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH:+${GAZEBO_RESOURCE_PATH}:}/opt/conda/share/gazebo/worlds' >> ~/.bashrc
RUN echo 'export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH:+${ROS_PACKAGE_PATHW}:}/opt/conda/share:/opt/conda/share/iCub/robots' >> ~/.bashrc
RUN echo 'export AMENT_PREFIX_PATH=${AMENT_PREFIX_PATH:+${AMENT_PREFIX_PATH}:}/opt/conda/share' >> ~/.bashrc
RUN echo 'export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH:+${GAZEBO_MODEL_PATH}:}/opt/conda/share/gazebo/models:/opt/conda/share/iCub/robots:/opt/conda/share' >> ~/.bashrc

# Launch bash from /home/user
WORKDIR /home/user
CMD ["bash"]
