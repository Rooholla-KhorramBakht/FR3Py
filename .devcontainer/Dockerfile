FROM ros:humble
ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get install -y -qq --no-install-recommends \ 
                                      libglvnd-dev \
                                      libgl1-mesa-dev \
                                      libegl1-mesa-dev \
                                      libgles2-mesa-dev \
                                      libxext6 \
                                      libx11-6 \
                                      freeglut3-dev \
                                      git \
                                      python3-pip \
                                      python3-tk \
                                      curl \
                                      vim \ 
                                      libcgal-dev \
                                      libcgal-demo \
                                      libglib2.0-dev \ 
                                      vim \ 
                                      libpoco-dev \
                                      libeigen3-dev \
                                      && rm -rf /var/lib/apt/lists/*
                                      
RUN curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | bash
RUN apt-get install git-lfs

# Install Python dependencies
RUN pip3 install scipy ipykernel warp-lang
RUN pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
RUN pip3 install matplotlib opencv-python proxsuite
RUN pip3 install isort black
RUN pip3 install scikit-learn casadi mujoco pin


# Compile and Install LCM
RUN cd /home/ && git clone https://github.com/lcm-proj/lcm.git && cd lcm && mkdir build && cd build && cmake .. && make -j"$(nproc)" && make install
RUN echo "export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3.8/site-packages" > /root/.bashrc
RUN echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/" > /root/.bashrc

# Compile and install libfranka
WORKDIR /root
RUN git clone --recursive https://github.com/frankaemika/libfranka --branch 0.13.3 && cd libfranka && mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF .. && cmake --build . && cpack -G DEB && dpkg -i libfranka*.deb 

# Install the FR3Py components
WORKDIR /home
RUN git clone --recursive https://github.com/Rooholla-KhorramBakht/FR3Py.git && cd FR3Py && python3 -m pip install -e . && cd fr3_gripper/ && python3 -m pip install .
RUN cd /home/FR3Py/fr3_bridge && mkdir build && cd build && cmake .. && make -j4 && make install
# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
