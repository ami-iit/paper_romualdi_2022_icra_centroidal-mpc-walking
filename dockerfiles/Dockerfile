FROM nvidia/opengl:1.2-glvnd-devel-ubuntu20.04
LABEL org.opencontainers.image.title="CentroidalMPC Docker Image"
LABEL org.opencontainers.image.description="Infrastructure for reproducing CentroidalMPC experiments"
LABEL org.opencontainers.image.source="https://raw.githubusercontent.com/ami-iit/centroidal-mpc-walking/main/dockerfiles/Dockerfile"
LABEL org.opencontainers.image.authors="Giulio Romualdi <giulio.romualdi@iit.it>"

ARG USERNAME=user
ARG USERID=1000

SHELL ["/bin/bash", "-c"]

# Non-interactive installation mode
ENV DEBIAN_FRONTEND=noninteractive

# Update apt database
RUN apt update

# Set the locale
RUN apt install -y -qq apt-utils locales && rm -rf /var/lib/apt/lists/*
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Install essentials
COPY deps.sh .
RUN  chmod +x ./deps.sh
RUN ./deps.sh && rm ./deps.sh && rm -rf /var/lib/apt/lists/*

# install gazebo
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add - && \
    apt update && \
    apt install -y -qq gazebo11 libgazebo11-dev && \
    rm -rf /var/lib/apt/lists/*

# Create the user
RUN useradd --create-home -s /bin/bash --no-user-group -u $USERID $USERNAME && \
    adduser $USERNAME sudo && \
    echo "$USERNAME ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

USER $USERNAME
WORKDIR /home/$USERNAME

# Get coinbrew
RUN mkdir CoinIpopt &&\
    cd CoinIpopt &&\
    wget https://raw.githubusercontent.com/coin-or/coinbrew/1cd6e53a9c6f3b235939da3ebf6cabaa14da44bd/coinbrew &&\
    chmod u+x coinbrew && \
    ./coinbrew build Ipopt --main-proj-version=releases/3.13.4 --prefix=install --test --no-prompt --verbosity=3 && \
    ./coinbrew install Ipopt --no-prompt && \
    rm -rf build

RUN echo 'export IPOPT_DIR=/home/$USERNAME/CoinIpopt/install' >> /home/$USERNAME/.bashrc && \
    echo 'export PKG_CONFIG_PATH=${PKG_CONFIG_PATH}:${IPOPT_DIR}/lib/pkgconfig' >> /home/$USERNAME/.bashrc && \
    echo 'export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${IPOPT_DIR}/lib' >> /home/$USERNAME/.bashrc && \
    echo 'export PATH=${PATH}:${IPOPT_DIR}/lib' >> /home/$USERNAME/.bashrc

RUN git config --global user.name user && \
    git config --global user.email user.user@user.it
ENV IPOPT_DIR=/home/$USERNAME/CoinIpopt/install
ENV PKG_CONFIG_PATH=${PKG_CONFIG_PATH}:${IPOPT_DIR}/lib/pkgconfig
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${IPOPT_DIR}/lib
ENV PATH=${PATH}:${IPOPT_DIR}/lib

COPY icub-models.patch .
RUN git clone https://github.com/robotology/robotology-superbuild.git && \
    cd robotology-superbuild && \
    git checkout v2021.08.1 && \
    mkdir -p build && cd build &&\
    cmake -GNinja \
    -DCMAKE_BUILD_TYPE=Release \
    -DNON_INTERACTIVE_BUILD:BOOL=ON \
    -DROBOTOLOGY_ENABLE_CORE:BOOL=ON \
    -DROBOTOLOGY_ENABLE_DYNAMICS:BOOL=ON \
    -DROBOTOLOGY_USES_GAZEBO:BOOL=ON \
    -DROBOTOLOGY_USES_OCTAVE:BOOL=OFF \
    -DROBOTOLOGY_USES_PYTHON:BOOL=OFF \
    -DROBOTOLOGY_USES_MATLAB:BOOL=OFF \
    -DROBOTOLOGY_ENABLE_DYNAMICS_FULL_DEPS:BOOL=ON .. && \
    cmake --build . && \
    cd ../src/bipedal-locomotion-framework && \
    git checkout 6cfe29fcfdd5c598bb0baa809b91c96139e4bcf1 && \
    cd ../icub-models && \
    cp /home/$USERNAME/icub-models.patch . && \
    git apply ./icub-models.patch && \
    cd ../../build/src/bipedal-locomotion-framework && \
    cmake . && cmake --build . --target install && \
    cd ../icub-models && \
    cmake . && cmake --build . --target install && \
    cd /home/${USERNAME}/robotology-superbuild/build && \
    find . -mindepth 1 ! -regex '^./install\(/.*\)?' -delete

RUN echo "source /home/$USERNAME/robotology-superbuild/build/install/share/robotology-superbuild/setup.sh" >> /home/$USERNAME/.bashrc && \
    echo 'export YARP_ROBOT_NAME=iCubGazeboV3' >> /home/${USERNAME}/.bashrc
RUN source /home/$USERNAME/robotology-superbuild/build/install/share/robotology-superbuild/setup.sh

# install the centroidal mpc
RUN git clone https://github.com/ami-iit/centroidal-mpc-walking.git && \
    cd centroidal-mpc-walking && \
    mkdir build && \
    cd build && \
    cmake -GNinja -DCMAKE_INSTALL_PREFIX=/home/${USERNAME}/robotology-superbuild/build/install .. && \
    cmake --build . --target install && \
    cd .. && \
    rm -rf build

RUN mkdir -p /home/${USERNAME}/.config/tmuxinator
COPY centroidal-mpc.yml /home/${USERNAME}/.config/tmuxinator/.

CMD ["tmuxinator", "start", "centroidal-mpc"]
