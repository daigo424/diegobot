COMPOSE_PJ_NAME    := diegobot
UNAME_M            := $(shell uname -m)
# docker-compose.gpu-*.ymlは開発機(x86_64、GUI/シミュレーション用イメージ)専用のoverride。
# ラズパイ実機(arm64)にはGPUパススルーが不要な上、BASE_IMAGE=osrf/ros:jazzy-desktopはamd64
# ビルドしか無くarm64では動かないため、arm64では一切適用しない(=Dockerfile側のデフォルト
# であるros:jazzy-ros-baseのままビルドされる)。
ifeq ($(UNAME_M),$(filter $(UNAME_M),aarch64 arm64))
GPU_COMPOSE_FILE   :=
# WSL2かネイティブLinuxかでGPUパススルーの構成が別物になる(edge/docker/docker-compose.gpu-*.yml参照)
# ため、/proc/versionで自動判定してoverrideファイルを差し替える。
else ifneq ($(shell grep -qi microsoft /proc/version 2>/dev/null && echo wsl),)
GPU_COMPOSE_FILE   := edge/docker/docker-compose.gpu-wsl.yml
else
GPU_COMPOSE_FILE   := edge/docker/docker-compose.gpu-native.yml
endif
# 無いデバイスパスをdocker composeにマウントさせようとするとコンテナ起動自体が失敗するため、
# 実際に接続されているハードウェアだけをoverrideで足す。LiDAR/micro-ROS/アームは.envのデバイスパスが
# 存在するかで自動判定する。ROBOT_ATTACHED=0で全て外す(個別に*_ATTACHED=0/1で上書きも可)。
ROBOT_ATTACHED     ?= 1
_HOST_DEVICE_EXISTS = $(if $(filter 1,$(ROBOT_ATTACHED)),$(shell . ./.env 2>/dev/null; [ -e "$${$(1)}" ] && echo 1 || echo 0),0)
LIDAR_FRONT_RIGHT_ATTACHED ?= $(call _HOST_DEVICE_EXISTS,HOST_LIDAR_FRONT_RIGHT_PORT)
LIDAR_REAR_LEFT_ATTACHED   ?= $(call _HOST_DEVICE_EXISTS,HOST_LIDAR_REAR_LEFT_PORT)
MICRO_ROS_ATTACHED ?= $(call _HOST_DEVICE_EXISTS,HOST_MICRO_ROS_PORT)
ARM_ATTACHED       ?= $(call _HOST_DEVICE_EXISTS,HOST_ARM_PORT)
# HP60Cは機体ごとに変わるパスが無いため、USBのベンダー/プロダクトIDで判定する。
HP60C_ATTACHED     ?= $(if $(filter 1,$(ROBOT_ATTACHED)),$(shell lsusb -d 3482:6723 >/dev/null 2>&1 && echo 1 || echo 0),0)
LIDAR_FRONT_RIGHT_COMPOSE_FILE := $(if $(filter 1,$(LIDAR_FRONT_RIGHT_ATTACHED)),edge/docker/docker-compose.lidar-front-right-attached.yml,)
LIDAR_REAR_LEFT_COMPOSE_FILE   := $(if $(filter 1,$(LIDAR_REAR_LEFT_ATTACHED)),edge/docker/docker-compose.lidar-rear-left-attached.yml,)
MICRO_ROS_COMPOSE_FILE := $(if $(filter 1,$(MICRO_ROS_ATTACHED)),edge/docker/docker-compose.micro-ros-attached.yml,)
ARM_COMPOSE_FILE       := $(if $(filter 1,$(ARM_ATTACHED)),edge/docker/docker-compose.arm-attached.yml,)
HP60C_COMPOSE_FILE     := $(if $(filter 1,$(HP60C_ATTACHED)),edge/docker/docker-compose.hp60c-attached.yml,)
BASE_COMPOSE_FILE      := $(if $(filter 1,$(LIDAR_FRONT_RIGHT_ATTACHED) $(LIDAR_REAR_LEFT_ATTACHED) $(MICRO_ROS_ATTACHED) $(ARM_ATTACHED) $(HP60C_ATTACHED)),edge/docker/docker-compose.base.yml,)

# 物体検出等のサーバー側処理用コンテナ(NVIDIA GPU前提)。ラズパイでは動かさない。
ifeq ($(UNAME_M),$(filter $(UNAME_M),aarch64 arm64))
PERCEPTION_COMPOSE_FILE :=
else
PERCEPTION_COMPOSE_FILE := edge/docker/docker-compose.perception.yml
endif

COMPOSE            := docker compose -f edge/docker/docker-compose.yml $(if $(GPU_COMPOSE_FILE),-f $(GPU_COMPOSE_FILE)) $(if $(PERCEPTION_COMPOSE_FILE),-f $(PERCEPTION_COMPOSE_FILE)) $(if $(LIDAR_FRONT_RIGHT_COMPOSE_FILE),-f $(LIDAR_FRONT_RIGHT_COMPOSE_FILE)) $(if $(LIDAR_REAR_LEFT_COMPOSE_FILE),-f $(LIDAR_REAR_LEFT_COMPOSE_FILE)) $(if $(MICRO_ROS_COMPOSE_FILE),-f $(MICRO_ROS_COMPOSE_FILE)) $(if $(ARM_COMPOSE_FILE),-f $(ARM_COMPOSE_FILE)) $(if $(HP60C_COMPOSE_FILE),-f $(HP60C_COMPOSE_FILE)) $(if $(BASE_COMPOSE_FILE),-f $(BASE_COMPOSE_FILE)) -p $(COMPOSE_PJ_NAME)
RUN                := $(COMPOSE) run --rm --remove-orphans
EXEC               := $(COMPOSE) exec
ROS2_SERVICE       := diegobot
ROS2_CONTAINER     := diegobot_container
ROS2_WS            := /workspace
ROS_DISTRO         := jazzy
ROS_INSTALL_PREFIX := /opt/ros/$(ROS_DISTRO)
MOBILE_DIR         := mobile
PKG                ?=
PKG_NAME           := $(if $(PKG),$(notdir $(PKG)),)
# サーバー側でだけ動かすパッケージ。ラズパイではビルド・依存導入の対象から外す
# (edge/docker/Dockerfileのrosdep段にも同じ除外がある)。
SERVER_PKGS        := amr_perception amr_moveit_config amr_manipulation amr_behaviors
PI_WS_SRC_PATHS    := $(patsubst edge/workspace/%,%,$(filter-out $(addprefix edge/workspace/src/,$(SERVER_PKGS)),$(wildcard edge/workspace/src/*)))
ifeq ($(UNAME_M),$(filter $(UNAME_M),aarch64 arm64))
WS_SRC_PATHS       := $(PI_WS_SRC_PATHS)
# ラズパイではPCL/rclcppを含むC++の1ファイルのコンパイルにDebugで約1.5GB、Releaseで約1GB要り、
# 並列に走らせたりDebugにするとスワップで固まる。パッケージは1つずつ、コンパイラはBUILD_JOBS個まで、Releaseでビルドする。
BUILD_JOBS         ?= 1
COLCON_BUILD_TYPE  := Release
COLCON_BUILD_LIMIT := MAKEFLAGS=-j$(BUILD_JOBS)
COLCON_BUILD_LIMIT_ARGS := --parallel-workers 1
else
WS_SRC_PATHS       := src
COLCON_BUILD_TYPE  := Debug
endif
COLCON_SELECT      := $(if $(PKG_NAME),--packages-select $(PKG_NAME),)
CMD_ROS2_SOURCE    := source $(ROS_INSTALL_PREFIX)/setup.bash && source /opt/ros2_controllers_ws/install/setup.bash && source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
CMD_ROS2_WS_SOURCE := test -f $(ROS2_WS)/install/setup.bash && source $(ROS2_WS)/install/setup.bash || true

up:
	@test -f .env || cp .env.example .env
	$(COMPOSE) --env-file .env up -d
	@. ./.env 2>/dev/null;
	$(MAKE) install-packages

# カメラ・LiDAR・Picoのいずれも未接続な開発機から、ラズパイ実機のtopicをrviz2等で
# 見るだけの用途向け。
up-remote:
	$(MAKE) up ROBOT_ATTACHED=0

# 開発機にPicoだけをUSB接続してmicro-ROSファームウェアを検証する用途向け
# (カメラ・LiDARは実機側のものなので開発機には無い)。
up-pico-remote:
	$(MAKE) up ROBOT_ATTACHED=0 MICRO_ROS_ATTACHED=1

# ラズパイ起動時にsystemd(edge/provisioning/diegobot.service.template)から呼ばれる。
# Wi-Fi未接続でも起動できるようinstall-packagesは行わない。
up-boot:
	@test -f .env || cp .env.example .env
	$(COMPOSE) --env-file .env up -d

down:
	$(COMPOSE) --env-file .env down

build:
	$(COMPOSE) --env-file .env build

login:
	$(EXEC) $(ROS2_SERVICE) bash

# サーバー側の処理(点群生成・物体検出)を起動する。ノード本体はdiegobotコンテナでcolcon buildしたものを使う。
# MODELで検出モデルを差し替えられる(例: make perception MODEL=yoloe-26s-seg-pf.engine)。
MODEL ?=
perception:
	$(EXEC) perception bash -c "source $(ROS_INSTALL_PREFIX)/setup.bash && source $(ROS2_WS)/install/local_setup.bash && ros2 launch amr_perception perception.launch.py $(if $(MODEL),model:=$(MODEL))"

# 実機のアームを開発機のMoveItで計画し、カメラで見つけた物をつかむ・置くアクション(/manipulator/pick, /manipulator/place)を出す。
# 物を探すため、先にmake perceptionも起動しておく。
manipulation:
	$(EXEC) $(ROS2_SERVICE) bash -c "$(CMD_ROS2_SOURCE) && $(CMD_ROS2_WS_SOURCE) && ros2 launch amr_manipulation manipulation.launch.py $(if $(RVIZ),use_rviz:=true)"

# manipulationに加え、探す・近づくアクション(/behavior_server/search, /behavior_server/approach)を出す。
# 車体が動くため、周りに物が無い床で使う。先にmake perceptionも起動しておく。
behaviors:
	$(EXEC) $(ROS2_SERVICE) bash -c "$(CMD_ROS2_SOURCE) && $(CMD_ROS2_WS_SOURCE) && ros2 launch amr_behaviors behaviors.launch.py $(if $(RVIZ),use_rviz:=true)"

# 実機のロボットモデル・TF・センサーを開発機のRVizで見る。
view-robot:
	$(EXEC) $(ROS2_SERVICE) bash -c "$(CMD_ROS2_SOURCE) && $(CMD_ROS2_WS_SOURCE) && ros2 launch amr_bringup view_robot.launch.xml"

# ros2 launchで起動したプロセスをCtrl+Cせず再起動すると、gz sim(rubyラッパー)や
# ros_gz_bridge系ノードが孤児化して残り続けることがある(SIGINTがlaunchの子孫全員に
# 伝播しないケース)。それらをまとめてSIGKILLで掃除する。
# docker-compose.ymlのpid: hostによりホストとPID空間を共有しているため、コンテナ内の
# プロセスとホスト側の一般プロセス(make自体等)が同じUIDで見分けが付かない。
# 代わりにdocker topでこのコンテナに属するPIDだけを正確に特定してkillする。
KILL_ROS_PATTERN := (ros2|gz sim|gzserver|gzclient|ros_gz|rviz2|robot_state_publisher|nav2|component_container|ascamera_node|teleop_twist_keyboard|ruby.*gz_tools)
kill-ros:
	@pids="$$(docker top $(ROS2_CONTAINER) -eo pid,args 2>/dev/null | grep -E '$(KILL_ROS_PATTERN)' | awk '{print $$1}' | tr '\n' ' ')"; \
	if [ -n "$$pids" ]; then \
	  $(EXEC) $(ROS2_SERVICE) bash -c "kill -9 $$pids" || true; \
	fi

# 次の行の `-e CMAKE_PREFIX_PATH=...` について:
# iceoryx_binding_c等、amentに登録されない素のCMakeパッケージ($(ROS_INSTALL_PREFIX)/lib/<arch>/cmake/配下)は
# setup.bashだけではCMAKE_PREFIX_PATHに入らないため、execで直接環境変数として注入する。
colcon:
	$(EXEC) -e CMAKE_PREFIX_PATH=$(ROS_INSTALL_PREFIX) $(ROS2_SERVICE) bash -c \
	  "$(CMD_ROS2_SOURCE) && $(CMD_ROS2_WS_SOURCE) && \
	   cd $(ROS2_WS) && $(CMD_RUN)"

# bear(edge/workspace/.clangdが参照する/workspace/compile_commands.jsonを生成)。
# --appendが無いと--packages-select時に他パッケージ分が上書きで消えるため必須。
# CLEAN=1でbuild/install/log/compile_commands.jsonを削除してからビルドする。
COLCON_BUILD_CLEAN := $(if $(CLEAN),rm -rf build/ install/ log/ compile_commands.json &&,)
COLCON_BUILD_WARN := -DCMAKE_WARN_DEPRECATED=$(if $(WARN),ON,OFF)
# status-は進行状況の行表示を切る。colcon-notification 0.3.2以降はこの表示の消去処理が再入して
# TypeErrorになり、ビルドが途中で打ち切られることがある。
colcon-build:
	$(MAKE) colcon CMD_RUN="$(COLCON_BUILD_CLEAN) $(COLCON_BUILD_LIMIT) bear --append -- colcon build --event-handlers status- $(COLCON_BUILD_LIMIT_ARGS) --symlink-install --base-paths $(WS_SRC_PATHS) --cmake-args -DCMAKE_BUILD_TYPE=$(COLCON_BUILD_TYPE) -DCMAKE_EXPORT_COMPILE_COMMANDS=ON $(COLCON_BUILD_WARN) --no-warn-unused-cli $(COLCON_SELECT)"
colcon-build-clean:
	$(MAKE) colcon-build CLEAN=1
colcon-build-clean-warn:
	$(MAKE) colcon-build CLEAN=1 WARN=1

# roscpp/rospy/catkinはvendor/angstrong/ascamera(HP60Cドライバ)のpackage.xmlに残るROS 1の宣言で、
# Jazzyでは解決できずrosdepが失敗するため除外する。
install-packages:
	$(MAKE) colcon CMD_RUN="sudo -E apt-get update && sudo -E rosdep install --from-paths $(WS_SRC_PATHS) --ignore-src -r -y --skip-keys='roscpp rospy catkin'"

# 詳細はedge/workspace/vendor.repos参照。
vendor-import:
	$(EXEC) $(ROS2_SERVICE) bash -c "cd $(ROS2_WS) && vcs import src < vendor.repos"

launch-urdf-display:
	$(MAKE) colcon CMD_RUN="ros2 launch urdf_tutorial display.launch.py model:=/workspace/src/my_robot_description/urdf/$(FILENAME)"

rqt-graph:
	$(MAKE) colcon CMD_RUN="rqt_graph"
launch-my-robot-description-display:
	$(MAKE) colcon CMD_RUN="ros2 launch my_robot_description display.launch.xml"

use-controller:
	$(MAKE) colcon CMD_RUN="ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel -p stamped:=true"

# シミュレーションのSO101の関節角[rad]。J1〜J6はshoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripperの順。
J1 ?= 0.0
J2 ?= 0.0
J3 ?= 0.0
J4 ?= 0.0
J5 ?= 0.0
J6 ?= 0.0
pub-arm-joints:
	$(MAKE) colcon CMD_RUN="ros2 topic pub -1 /arm_controller/commands std_msgs/msg/Float64MultiArray '{data: [$(J1), $(J2), $(J3), $(J4), $(J5), $(J6)]}'"
tf-tree-pdf:
	$(MAKE) colcon CMD_RUN="ros2 run tf2_tools view_frames"
tf-tree:
	$(MAKE) colcon CMD_RUN="ros2 run rqt_tf_tree rqt_tf_tree"
teleop-twist-keyboard:
	$(MAKE) colcon CMD_RUN="ros2 run teleop_twist_keyboard teleop_twist_keyboard"
PI_HOST ?= diegobot.local
PI_USER ?= daigo
pi-rsync:
	rsync -avz \
	  --exclude=edge/workspace/build/ \
	  --exclude=edge/workspace/install/ \
	  --exclude=edge/workspace/log/ \
	  --exclude=edge/workspace/build-arm64/ \
	  --exclude=edge/workspace/core \
	  --exclude=edge/workspace/core.* \
	  --include=edge/workspace/src/vendor/ \
	  --include=edge/workspace/src/vendor/angstrong/*** \
	  --exclude=edge/workspace/src/vendor/* \
	  ./ $(PI_USER)@$(PI_HOST):~/diegobot
	@echo "vendor/のうちvcstool管理の外部パッケージは転送していません。ラズパイ側で 'make vendor-import' を実行してください"
pi-ssh:
	ssh $(PI_USER)@$(PI_HOST)
pi-diagnose:
	ssh $(PI_USER)@$(PI_HOST) bash -s < edge/provisioning/scripts/pi-diagnose.sh
# 初回のラズパイにはmakeが無いことがあるため、ラズパイ上ではなく開発機からssh越しに実行する。
# -tはsudoのパスワード入力用。
pi-setup:
	ssh -t $(PI_USER)@$(PI_HOST) 'cd ~/diegobot/edge/provisioning && sudo ./setup.sh'
pi-setup-dev:
	ssh -t $(PI_USER)@$(PI_HOST) 'cd ~/diegobot/edge/provisioning && sudo ./setup.dev.sh'
FILE ?=
pi-fetch:
	scp $(PI_SSH_OPTS) $(PI_USER)@$(PI_HOST):$(FILE) .

# ラズパイ上でのイメージビルドが遅い/メモリ不足になりがちな場合に、開発機側でarm64向けに
# クロスビルドしてイメージそのものを転送する。QEMUエミュレーション経由のためネイティブ
# ビルドより遅くなり得るが、ラズパイのCPU/RAMやネットワーク帯域(apt-get/git clone)を使わずに済む。
# diegobot:latestではなく専用タグにする(makeをdiegobot:latestで上書きしてしまい、
# 開発機のmake upが誤ってarm64イメージを掴むのを防ぐため)。
PI_IMAGE_TAG := diegobot:latest-arm64
PI_IMAGE_TAR := /tmp/diegobot-arm64.tar.gz

# QEMU binfmtエミュレーションを登録し、x86ホストでarm64コンテナをビルドできるようにする。
ensure-arm64-emulation:
	docker run --privileged --rm tonistiigi/binfmt --install arm64

pi-build-image: ensure-arm64-emulation
	docker buildx build --platform linux/arm64 -f edge/docker/Dockerfile --build-context workspace=edge/workspace -t $(PI_IMAGE_TAG) --load edge/docker

# rsyncとsshを別々に叩くと毎回パスワード入力が要るため、ControlMasterで最初の接続を
# 使い回す(ControlPersistの間はマスター接続が生きたままになり、以降のrsync/sshはパスワード不要)。
PI_SSH_OPTS := -o ControlMaster=auto -o ControlPath=/tmp/ssh-diegobot-%r@%h:%p -o ControlPersist=10m
pi-push-image: pi-build-image
	docker save $(PI_IMAGE_TAG) | gzip > $(PI_IMAGE_TAR)
	rsync -avz --progress -e "ssh $(PI_SSH_OPTS)" $(PI_IMAGE_TAR) $(PI_USER)@$(PI_HOST):/tmp/
	ssh $(PI_SSH_OPTS) $(PI_USER)@$(PI_HOST) '\
	  docker load < $(PI_IMAGE_TAR) && \
	  docker tag $(PI_IMAGE_TAG) diegobot:latest && \
	  rm -f $(PI_IMAGE_TAR) && \
	  cd ~/diegobot && \
	  make up-boot \
	'
	rm -f $(PI_IMAGE_TAR)

# ラズパイでC++パッケージをビルドするとメモリ不足で止まるため、開発機でarm64イメージを使って
# ビルドし、PKGのinstallだけをラズパイの同じ場所へ送る。PI_IMAGE_TAGはラズパイのイメージと同じ版であること。
# --symlink-installを付けない(リンク先のbuild/がラズパイに無いため)。送った後は、ラズパイ側でコンテナを再起動する。
PI_CROSS_DIR := edge/workspace/build-arm64
pi-cross-build: ensure-arm64-emulation
	@test -n "$(PKG_NAME)" || { echo "PKG=<パッケージ名> を指定してください"; exit 1; }
	mkdir -p $(PI_CROSS_DIR)/build $(PI_CROSS_DIR)/install $(PI_CROSS_DIR)/log
	docker run --rm --platform linux/arm64 --user $$(id -u):$$(id -g) -e HOME=/tmp \
	  -v $(CURDIR)/edge/workspace/src:$(ROS2_WS)/src:ro \
	  -v $(CURDIR)/$(PI_CROSS_DIR)/build:$(ROS2_WS)/build \
	  -v $(CURDIR)/$(PI_CROSS_DIR)/install:$(ROS2_WS)/install \
	  -v $(CURDIR)/$(PI_CROSS_DIR)/log:$(ROS2_WS)/log \
	  -w $(ROS2_WS) $(PI_IMAGE_TAG) bash -c "$(CMD_ROS2_SOURCE) && colcon build --event-handlers status- --base-paths $(PI_WS_SRC_PATHS) --cmake-args -DCMAKE_BUILD_TYPE=Release --no-warn-unused-cli --packages-up-to $(PKG_NAME)"
	rsync -az --delete -e "ssh $(PI_SSH_OPTS)" $(PI_CROSS_DIR)/install/$(PKG_NAME)/ $(PI_USER)@$(PI_HOST):~/diegobot/edge/workspace/install/$(PKG_NAME)/

# flutterはsnap経由でインストールされ/snap/binに入るが、呼び出し元シェルのPATHに
# /snap/binが無いと"flutter: not found"になるため、ここで明示的に足す。
android-install:
	cd $(MOBILE_DIR) && PATH="$$PATH:/snap/bin" flutter build apk --release && PATH="$$PATH:/snap/bin" flutter install
