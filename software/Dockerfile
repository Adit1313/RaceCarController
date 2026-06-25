# syntax=docker/dockerfile:1
FROM ros:noetic-ros-core-focal

# Install, on top of core, the desktop packages of ROS
RUN \
    --mount=type=cache,sharing=locked,target=/var/lib/apt/lists \
    --mount=type=cache,sharing=locked,target=/var/cache/apt \
    rm -f /etc/apt/apt.conf.d/docker-clean \
    && apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop=1.5.0-1*

# install bootstrap tools
RUN \
    --mount=type=cache,sharing=locked,target=/var/lib/apt/lists \
    --mount=type=cache,sharing=locked,target=/var/cache/apt \
    rm -f /etc/apt/apt.conf.d/docker-clean \
    && apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools

# bootstrap rosdep
RUN rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO

# -------- CRS setup --------

WORKDIR /code

# APT dependencies for the compilation of casadi/acados
RUN \
    --mount=type=cache,sharing=locked,target=/var/lib/apt/lists \
    --mount=type=cache,sharing=locked,target=/var/cache/apt \
    rm -f /etc/apt/apt.conf.d/docker-clean \
    && apt-get update && apt-get install -y --no-install-recommends \
    git \
    ccache \
    python3-pip \
    libeigen3-dev \
    # IPOPT for lighthouse_calib
    gfortran \
    coinor-libipopt-dev \
    libblas-dev \
    liblapack-dev \
    pkg-config \
    cm-super \
    dvipng \
    texlive-latex-extra \
    texlive-latex-recommended \
    python3-cffi

# Update setuptools version so setuptools-scm works
RUN pip3 install "setuptools>=61"

# Compile and install CasADi
COPY .setup/ubuntu/install_casadi.sh .setup/ubuntu/install_casadi.sh
RUN sudo ./.setup/ubuntu/install_casadi.sh

# Compile and install acados
COPY .setup/ubuntu/install_acados.sh .setup/ubuntu/install_acados.sh
RUN sudo ./.setup/ubuntu/install_acados.sh
ENV ACADOS_SOURCE_DIR="/acados"
ENV LD_LIBRARY_PATH="/acados/lib:$LD_LIBRARY_PATH"

# Setup the Tera renderer on Arm machines
COPY .setup/ubuntu/install_tera_renderer.sh .setup/ubuntu/install_tera_renderer.sh
RUN sudo ./.setup/ubuntu/install_tera_renderer.sh

# Other apt dependencies
RUN \
    --mount=type=cache,sharing=locked,target=/var/lib/apt/lists \
    --mount=type=cache,sharing=locked,target=/var/cache/apt \
    rm -f /etc/apt/apt.conf.d/docker-clean \
    && apt-get update && apt-get install -y --no-install-recommends \
    # Joystick support
    ros-noetic-joy \
    # Realitimetools for RT Controller
    ros-noetic-realtime-tools \
    # Plotjuggler
    ros-noetic-plotjuggler \
    ros-noetic-plotjuggler-ros \
    python3-tk \
    # required by Matplotlib
    libfreetype6-dev \
    # required by wifi_com
    protobuf-compiler \
    libprotobuf-dev \
    clang-format \
    clang-tidy \
    # GDB debugger
    gdb \
    valgrind \
    tmux

# Install core dependencies first (dirty fix for the pyzmq issue)
RUN pip3 install --upgrade wheel cython
RUN pip3 install numpy==1.24.4 scipy==1.10.1 PyYAML==6.0.2

# Install Python requirements, caching the installation
COPY requirements.txt requirements.txt
RUN --mount=type=cache,sharing=locked,target=/root/.cache \
    python3 -m pip install -r requirements.txt

# Install car plotting script
COPY ./src/scripts/car_plotter ./src/scripts/car_plotter
RUN pip3 install -e src/scripts/car_plotter

# Install catkin
RUN --mount=type=cache,sharing=locked,target=/root/.cache \
    pip3 install catkin_tools

COPY .crs/ .crs/
RUN sudo ./.crs/setup-crs.sh

# OPENMP SETTINGS
ENV OMP_NUM_THREADS=2
ENV OMP_PLACES=cores
ENV OMP_PROC_BIND=TRUE

# GTEST
COPY .setup/ubuntu/install_gtest.sh .setup/ubuntu/install_gtest.sh
RUN sudo ./.setup/ubuntu/install_gtest.sh

# Let git run pre-commit inside container
RUN git config --system --add safe.directory .

# Extend path so we can call scripts from anywhere
ENV PATH="/code/scripts:$PATH"

# MANIPULATE BASHRC
RUN echo "[[ -f /code/devel/setup.bash ]] && source /code/devel/setup.bash" >> ~/.bashrc

# Install L4acados
COPY .setup/ubuntu/install_l4acados.sh .setup/ubuntu/install_l4acados.sh
RUN sudo ./.setup/ubuntu/install_l4acados.sh

# ROS EOL ingore warning
ENV DISABLE_ROS1_EOL_WARNINGS=1
