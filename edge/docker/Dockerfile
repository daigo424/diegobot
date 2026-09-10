# syntax=docker/dockerfile:1
# ↑パーサーディレクティブはファイル最初の行でなければ無効になる(手前に他のコメントを
# 置くとBuildKitはディレクティブ探索を止める)。RUN ... <<EOF のheredoc構文を使うために必要。

# 実機(ラズパイ、arm64)用。osrf/ros:jazzy-desktop系はamd64ビルドしか配布されておらず
# arm64ではexec format errorになるため、公式のマルチアーキ(amd64/arm64)イメージを使う。
# GUI/シミュレーションツール込みの開発機向けビルドはDockerfile.localを使う。
FROM ros:jazzy-ros-base

# moveit/moveit-py/controller-manager/ros2-controlはここでは入れない。src/以下の各package.xmlに
# exec_depend/dependとして宣言済みで、コンテナ起動後のmake install-packages(rosdep)が拾う。
# ここに残すのはpackage.xmlで宣言しようがないもの(単体で叩くチュートリアル、ミドルウェア選択、開発ツール)だけ。
RUN apt-get update && apt-get install -y \
      ros-${ROS_DISTRO}-teleop-twist-keyboard \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
      ros-${ROS_DISTRO}-rosbridge-server \
      ros-${ROS_DISTRO}-ros2-control \
      ros-${ROS_DISTRO}-ros2-controllers \
      python3-colcon-common-extensions \
      python3-colcon-argcomplete \
      python3-rosdep \
      udev \
      pkg-config \
      git \
      bear \
      vim \
    && rm -rf /var/lib/apt/lists/*


ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN rosdep update

# apt版joint_trajectory_controller(4.40.1)は同じくapt版のcontroller_interface(4.45.2)とABI不整合でon_init()がSEGVする
# (packages.ros.org側の既知の不整合。ros-controls/ros2_control#2049)。ソースビルドで現行のcontroller_interfaceに合わせる。
RUN git clone --branch ${ROS_DISTRO} --depth 1 \
      https://github.com/ros-controls/ros2_controllers.git \
      /opt/ros2_controllers_ws/src/ros2_controllers

RUN apt-get update && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep install --from-paths /opt/ros2_controllers_ws/src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    cd /opt/ros2_controllers_ws && \
    colcon build --merge-install

# micro-ros-agentはjazzy含めどのdistroでもバイナリ(.deb)未リリース(index.ros.org上でRELEASEDだが各distro欄は空)。
# ソースビルド必須。depend先のmicro_ros_msgsはバイナリ提供されているのでrosdepが自動解決する。
RUN git clone --branch ${ROS_DISTRO} --depth 1 \
      https://github.com/micro-ROS/micro-ROS-Agent.git \
      /opt/micro_ros_agent_ws/src/micro-ROS-Agent

RUN apt-get update && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep install --from-paths /opt/micro_ros_agent_ws/src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    cd /opt/micro_ros_agent_ws && \
    colcon build --merge-install

# bash -c "... && bash"のような入れ子bash越しだとcomplete登録(colcon-argcomplete)が
# 子プロセスに引き継がれない(exportされる環境変数と違い、complete -Fはシェルのメモリ内状態のため)。
# 対話シェル自身の.bashrcで毎回sourceすることで、どの経路で入っても補完が効くようにする。
RUN grep -qxF '# --- ros2 shell setup (diegobot) ---' /root/.bashrc || \
    cat >> /root/.bashrc <<EOF
# --- ros2 shell setup (diegobot) ---
source /opt/ros/${ROS_DISTRO}/setup.bash
source /opt/ros2_controllers_ws/install/setup.bash
source /opt/micro_ros_agent_ws/install/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
test -f /workspace/install/setup.bash && source /workspace/install/setup.bash
EOF

ENV TURTLEBOT3_MODEL=waffle

WORKDIR /workspace
