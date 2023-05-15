# syntax=docker/dockerfile:1
FROM dustynv/ros:galactic-pytorch-l4t-r32.7.1

RUN add-apt-repository ppa:ubuntu-toolchain-r/test \
 && apt-get update && apt-get upgrade --yes \
 && cd /tmp && wget https://apt.llvm.org/llvm.sh \
 && chmod +x llvm.sh \
 && ./llvm.sh 16 \
 && rm -f llvm.sh \
 && rm -rf /var/lib/apt/lists/*

# Set Clang 16 as default compiler
ENV CC clang-16
ENV CXX clang++-16

RUN apt-get update && apt-get install --yes --no-install-recommends \
        ccache \
        python3.7 python3.7-dev \
 && python3.7 -m pip install -U pip \
 && python3.7 -m pip install meson ninja \
 && rm -rf /var/lib/apt/lists/*

# Set up Bash env
RUN <<EOF
printf '
PROMPT_COMMAND=\047printf "\\n"\047
PS1=\047\[\\e[0;36m\]\w\\n\[\\e[1;32m\]>>\[\\e[0m\] \047
' >> $HOME/.bashrc
EOF

RUN python3.7 -m pip install adafruit-circuitpython-motorkit Jetson.GPIO
