# https://docs.docker.com/engine/reference/builder/#syntax

FROM osrf/ros:humble-desktop-jammy AS base

SHELL ["/bin/bash", "-c"]

# Create the user
ARG USERNAME=elec330
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN <<-EOF
	groupadd --gid $USER_GID $USERNAME
	useradd --shell /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME
	apt-get update
	apt-get install -y sudo
	echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME
	chmod 0440 /etc/sudoers.d/$USERNAME
	rm -rf /var/lib/apt/lists/*
EOF
# Set the default user
USER $USERNAME

# Setup ROS env and enable colcon argcomplete feature and colcon_cd on every Bash
RUN <<EOF
printf '\n# Setup ROS env\n' >> $HOME/.bashrc
echo '. /opt/ros/$ROS_DISTRO/setup.bash' >> $HOME/.bashrc
printf '\n# Enable argcomplete of colcon\n' >> $HOME/.bashrc
echo '. /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash' >> $HOME/.bashrc
printf '\n# Enable colcon_cd\n' >> $HOME/.bashrc
echo '. /usr/share/colcon_cd/function/colcon_cd.sh' >> $HOME/.bashrc
echo 'export _colcon_cd_root=/opt/ros/$ROS_DISTRO/' >> $HOME/.bashrc
printf '\n' >> $HOME/.bashrc
EOF

# Make a colcon workspace
RUN <<EOF
mkdir -p $HOME/ws/src
echo '# Setup colcon workspace' >> $HOME/.bashrc
echo '[ -f $HOME/ws/install/local_setup.bash ] && . $HOME/ws/install/local_setup.bash' >> $HOME/.bashrc
printf '\n' >> $HOME/.bashrc
EOF

# Get Rust and enable tab completion of rustup and cargo for Bash
RUN <<-EOF
	curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
	source "$HOME/.cargo/env"
	mkdir -p $HOME/.local/share/bash-completion/completions
	rustup completions bash >> $HOME/.local/share/bash-completion/completions/rustup
	rustup completions bash cargo >> $HOME/.local/share/bash-completion/completions/cargo
EOF

RUN <<EOF
sudo apt-get update
sudo apt-get install --yes --no-install-recommends \
	python3-pip \
	bash-completion \
	libclang-dev
sudo rm -rf /var/lib/apt/lists/*
EOF
