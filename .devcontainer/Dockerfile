# https://docs.docker.com/engine/reference/builder/#syntax

FROM ros:humble-ros-base-jammy

RUN apt-get update && apt-get install --yes --no-install-recommends \
        bash-completion \
        curl wget \
        python3-pip \
        ros-$ROS_DISTRO-foxglove-bridge \
        ros-$ROS_DISTRO-cartographer-ros \
        ros-$ROS_DISTRO-hls-lfcd-lds-driver \
 && python3 -m pip install meson ninja \
 && rm -rf /var/lib/apt/lists/*

# Enable bash completion for root
RUN sed -i '97,99 {s/^#//}' ~/.bashrc

RUN echo 'deb http://apt.llvm.org/jammy/ llvm-toolchain-jammy main' | sudo tee /etc/apt/sources.list.d/llvm.list && \
    echo 'deb-src http://apt.llvm.org/jammy/ llvm-toolchain-jammy main' | sudo tee /etc/apt/sources.list.d/llvm.list && \
    wget -qO- https://apt.llvm.org/llvm-snapshot.gpg.key | sudo tee /etc/apt/trusted.gpg.d/apt.llvm.org.asc && \
    apt-get update && apt-get install --yes --no-install-recommends \
        # clang-15 \
        clangd-15 \
        # clang-format-15 \
        lldb-15 \
 && rm -rf /var/lib/apt/lists/*

# Set Clang as default compiler
# ENV CC clang-15
# ENV CXX clang++-15

# Get Rust and enable tab completion of rustup and cargo for Bash
RUN <<-EOF
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
    . $HOME/.cargo/env
    mkdir -p $HOME/.local/share/bash-completion/completions
    rustup completions bash >> $HOME/.local/share/bash-completion/completions/rustup
    rustup completions bash cargo >> $HOME/.local/share/bash-completion/completions/cargo
EOF

# Set up Bash env
RUN <<EOF
mkdir -p /ws
printf '
# Setup ROS env
. /opt/ros/$ROS_DISTRO/setup.bash

# Enable argcomplete of colcon
. /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

# Enable colcon_cd
. /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/$ROS_DISTRO/

# Setup colcon workspace
[ -f /ws/install/local_setup.bash ] && . /ws/install/local_setup.bash

PROMPT_COMMAND=\047printf "\\n"\047
PS1=\047\[\\e[0;36m\]\w\\n\[\\e[1;32m\]>>\[\\e[0m\] \047
' >> $HOME/.bashrc
EOF

RUN apt-get update && apt-get install --yes --no-install-recommends \
        # Speed up compilation
        ccache \
        # Required by R2R 
        libclang-dev \
 && rm -rf /var/lib/apt/lists/*
