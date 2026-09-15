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
# ROBOT_ATTACHED=0でPico/LiDARのUSBデバイス無しでも起動できる(開発機からラズパイのtopicを
# rviz2等で見るだけの用途向け)。デフォルトは実機接続前提の1。
ROBOT_ATTACHED     ?= 1
ROBOT_ATTACHED_COMPOSE_FILE := $(if $(filter 1,$(ROBOT_ATTACHED)),edge/docker/docker-compose.robot-attached.yml,)
COMPOSE            := docker compose -f edge/docker/docker-compose.yml $(if $(GPU_COMPOSE_FILE),-f $(GPU_COMPOSE_FILE)) $(if $(ROBOT_ATTACHED_COMPOSE_FILE),-f $(ROBOT_ATTACHED_COMPOSE_FILE)) -p $(COMPOSE_PJ_NAME)
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
COLCON_SELECT      := $(if $(PKG_NAME),--packages-select $(PKG_NAME),)
CMD_ROS2_SOURCE    := source $(ROS_INSTALL_PREFIX)/setup.bash && source /opt/ros2_controllers_ws/install/setup.bash && source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
CMD_ROS2_WS_SOURCE := test -f $(ROS2_WS)/install/setup.bash && source $(ROS2_WS)/install/setup.bash || true

up:
	@test -f .env || cp .env.example .env
	$(COMPOSE) --env-file .env up -d
	@. ./.env 2>/dev/null;
	$(MAKE) install-packages

up-remote:
	$(MAKE) up ROBOT_ATTACHED=0

down:
	$(COMPOSE) --env-file .env down

build:
	$(COMPOSE) --env-file .env build

login:
	$(EXEC) $(ROS2_SERVICE) bash

# ros2 launchで起動したプロセスをCtrl+Cせず再起動すると、gz sim(rubyラッパー)や
# ros_gz_bridge系ノードが孤児化して残り続けることがある(SIGINTがlaunchの子孫全員に
# 伝播しないケース)。それらをまとめてSIGKILLで掃除する。
# docker-compose.ymlのpid: hostによりホストとPID空間を共有しているため、
# 1) -u rootでコンテナ内(root実行)由来のものだけに絞り、ホスト側の一般ユーザープロセス
#    (make/docker compose exec自体など)を誤って巻き込まない
# 2) パターン中の各キーワードを[x]で1文字だけ字句分割し、pkill自身のコマンドライン
#    (-fの引数にこのパターン文字列がそのまま載る)に自己マッチしてpkillが自爆するのを防ぐ
KILL_ROS_PATTERN := ([r]os2|g[z] sim|gzs[e]rver|gzcl[i]ent|ros[_]gz|rvi[z]2|robot_state_publ[i]sher|[n]av2|component_conta[i]ner|teleop_twist_key[b]oard|rub[y].*gz_tools)
kill-ros:
	$(EXEC) $(ROS2_SERVICE) bash -c \
	  "pkill -9 -u root -f '$(KILL_ROS_PATTERN)' || true"

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
colcon-build:
	$(MAKE) colcon CMD_RUN="$(COLCON_BUILD_CLEAN) bear --append -- colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON $(COLCON_BUILD_WARN) --no-warn-unused-cli $(COLCON_SELECT)"
colcon-build-clean:
	$(MAKE) colcon-build CLEAN=1
colcon-build-clean-warn:
	$(MAKE) colcon-build CLEAN=1 WARN=1

# libcameraはedge/docker/Dockerfileでソースビルド済み(apt版と共存不可)のため、
# rosdepがapt版を入れないよう--skip-keysで除外する。
install-packages:
	$(MAKE) colcon CMD_RUN="apt-get update && rosdep install --from-paths src --ignore-src -r -y --skip-keys=libcamera"

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

J1 ?= 0.0
J2 ?= 0.0
pub-arm-joints:
	$(MAKE) colcon CMD_RUN="ros2 topic pub -1 /arm_joints_controller/commands std_msgs/msg/Float64MultiArray '{data: [$(J1), $(J2)]}'"
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
	  --exclude=edge/workspace/core \
	  --exclude=edge/workspace/core.* \
	  ./ $(PI_USER)@$(PI_HOST):~/diegobot
pi-ssh:
	ssh $(PI_USER)@$(PI_HOST)
pi-diagnose:
	ssh $(PI_USER)@$(PI_HOST) bash -s < edge/provisioning/scripts/pi-diagnose.sh
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
	docker buildx build --platform linux/arm64 -f edge/docker/Dockerfile -t $(PI_IMAGE_TAG) --load edge/docker

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
	  docker compose -f edge/docker/docker-compose.yml -p diegobot --env-file .env up -d --force-recreate \
	'
	rm -f $(PI_IMAGE_TAR)

# flutterはsnap経由でインストールされ/snap/binに入るが、呼び出し元シェルのPATHに
# /snap/binが無いと"flutter: not found"になるため、ここで明示的に足す。
android-install:
	cd $(MOBILE_DIR) && PATH="$$PATH:/snap/bin" flutter build apk --release && PATH="$$PATH:/snap/bin" flutter install
