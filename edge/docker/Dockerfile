# syntax=docker/dockerfile:1
# ↑パーサーディレクティブはファイル最初の行でなければ無効になる(手前に他のコメントを
# 置くとBuildKitはディレクティブ探索を止める)。RUN ... <<EOF のheredoc構文を使うために必要。

# edge/workspace/src(compose/Makefileがworkspaceという名前付きビルドコンテキストとして渡す)から
# package.xmlだけを抜き出す。src全体を持ち込むと、ソースを触るたびに後段のrosdep installが再実行されるため。
# サーバー側でだけ動かすパッケージ(MakefileのSERVER_PKGS)の依存は入れない。
FROM ros:jazzy-ros-base AS workspace-package-xml
COPY --from=workspace src /src
RUN mkdir /out && cd /src && find . \( -path ./amr_perception -o -path ./amr_moveit_config \) -prune -o -name package.xml -exec cp --parents {} /out/ \;

# 実機(ラズパイ、arm64)用。osrf/ros:jazzy-desktop系はamd64ビルドしか配布されておらず
# arm64ではexec format errorになるため、公式のマルチアーキ(amd64/arm64)イメージを使う。
# GUI/シミュレーションツール込みの開発機向けビルドはDockerfile.localを使う。
FROM ros:jazzy-ros-base

# moveit/moveit-py/controller-manager/ros2-controlはここでは入れない。src/以下の各package.xmlに
# exec_depend/dependとして宣言済みで、後段のrosdep installが拾う。
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
      meson \
      ninja-build \
      libyaml-dev \
      python3-yaml \
      python3-ply \
      python3-jinja2 \
      libudev-dev \
      openssl \
    && rm -rf /var/lib/apt/lists/*


ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN rosdep update

# apt版joint_trajectory_controller(4.40.1)は同じくapt版のcontroller_interface(4.45.2)とABI不整合でon_init()がSEGVする
# (packages.ros.org側の既知の不整合。ros-controls/ros2_control#2049)。ソースビルドで現行のcontroller_interfaceに合わせる。
# ブランチ先頭に追従すると、aptのrealtime_toolsとAPIがずれてビルドが壊れる(JTCのtrySetFeedback)
# ため、動作確認済みのコミットに固定する。
RUN git init /opt/ros2_controllers_ws/src/ros2_controllers && \
    cd /opt/ros2_controllers_ws/src/ros2_controllers && \
    git fetch --depth 1 https://github.com/ros-controls/ros2_controllers.git 9b3ee3fcbee3cc524e65e770dca97732d1c14e02 && \
    git checkout FETCH_HEAD

RUN apt-get update && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep install --from-paths /opt/ros2_controllers_ws/src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    cd /opt/ros2_controllers_ws && \
    colcon build --merge-install

# micro-ros-agentはjazzy含めどのdistroでもバイナリ(.deb)未リリース(index.ros.org上でRELEASEDだが各distro欄は空)。
# ソースビルド必須。depend先のmicro_ros_msgsはバイナリ提供されているのでrosdepが自動解決する。
# ブランチ先頭の変更でビルドが突然壊れないよう、動作確認済みのコミットに固定する。
RUN git init /opt/micro_ros_agent_ws/src/micro-ROS-Agent && \
    cd /opt/micro_ros_agent_ws/src/micro-ROS-Agent && \
    git fetch --depth 1 https://github.com/micro-ROS/micro-ROS-Agent.git 7c932329ad5591ef23942ef1962534258c16b000 && \
    git checkout FETCH_HEAD

RUN apt-get update && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep install --from-paths /opt/micro_ros_agent_ws/src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    cd /opt/micro_ros_agent_ws && \
    colcon build --merge-install

# apt版libcamera(ros-${ROS_DISTRO}-libcamera、camera_rosが依存)は、imx708のAfWindows制御を
# IPAプロキシへシリアライズする際にクラッシュする(FATAL: A list of V4L2 controls requires
# a ControlInfoMap)。ホストのrpicam-appsで動いているRaspberry Pi財団パッチ版libcameraを
# ソースビルドし、/usr/localにインストールする(apt版と共存させると意図せずapt版がリンク
# されるため、camera_rosをビルドするedge/workspace/vendor.repos側ではlibcamera/camera-ros
# のapt版を一切インストールしないこと)。
RUN git clone --branch v0.7.2+rpt20260817 --depth 1 \
      https://github.com/raspberrypi/libcamera.git \
      /opt/libcamera

RUN cd /opt/libcamera && \
    meson setup build --prefix=/usr/local --buildtype=release \
      -Dgstreamer=disabled -Dpycamera=disabled -Dqcam=disabled -Dcam=disabled -Ddocumentation=disabled && \
    ninja -C build install && \
    ldconfig

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

# roscpp/rospy/catkinはvendor/angstrong/ascamera(HP60Cドライバ)のpackage.xmlに残るROS 1の宣言で、
# Jazzyでは解決できずrosdepが失敗するため除外する(ビルド自体はament_cmakeで行われ不要)。
COPY --from=workspace-package-xml /out /tmp/workspace_src
RUN apt-get update && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep install --from-paths /tmp/workspace_src --ignore-src -r -y --skip-keys="libcamera roscpp rospy catkin" && \
    rm -rf /var/lib/apt/lists/*

ENV TURTLEBOT3_MODEL=waffle

WORKDIR /workspace

# ホスト側とUID/GIDを合わせることで、bind mountされたsrc/にコンテナ内から書き込んだ
# ファイルがホスト側から見てroot所有になる(≒ホスト側では書き込めなくなる)のを防ぐ。
# apt-get/rosdep等システム権限が必要な操作は、実行時にdocker compose exec -u rootで
# 個別に上書きする(Makefile参照)。ベースイメージ(Ubuntu 24.04)がUID/GID 1000に
# 標準で"ubuntu"ユーザーを用意しているため、新規作成ではなくリネームする。
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupmod -n app -g ${USER_GID} ubuntu && \
    usermod -l app -u ${USER_UID} -d /home/app -m -g app ubuntu && \
    apt-get update && apt-get install -y sudo && rm -rf /var/lib/apt/lists/* && \
    echo 'app ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/app && \
    chmod 0440 /etc/sudoers.d/app && \
    cp /root/.bashrc /home/app/.bashrc && \
    cp -r /root/.ros /home/app/.ros && \
    chown -R app:app /home/app /workspace

USER app
