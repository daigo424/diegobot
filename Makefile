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
# 実際に接続されているハードウェアだけをoverrideで足す。micro-ROS/アームはedge/.envのデバイスパスが
# 存在するかで、LiDARは本体のシリアル番号が見つかるかで自動判定する。ROBOT_ATTACHED=0で全て外す(個別に*_ATTACHED=0/1で上書きも可)。
ROBOT_ATTACHED     ?= 1
# edge(ロボット側のROS一式)のコンテナが使う環境変数(デバイスパス・ROS_DOMAIN_ID等)。
EDGE_ENV           := edge/.env
_HOST_DEVICE_EXISTS = $(if $(filter 1,$(ROBOT_ATTACHED)),$(shell . ./$(EDGE_ENV) 2>/dev/null; [ -e "$${$(1)}" ] && echo 1 || echo 0),0)
# LiDARの変換基板は2台とも同じシリアル番号で見分けられないため、本体のシリアル番号から挿さっているポートを探す。
_LIDAR_PORTS := $(if $(filter 1,$(ROBOT_ATTACHED)),$(shell . ./$(EDGE_ENV) 2>/dev/null; python3 edge/provisioning/scripts/find_lidar_ports.py "$$LIDAR_FRONT_RIGHT_SERIAL" "$$LIDAR_REAR_LEFT_SERIAL" 2>/dev/null))
export HOST_LIDAR_FRONT_RIGHT_PORT := $(word 1,$(_LIDAR_PORTS))
export HOST_LIDAR_REAR_LEFT_PORT   := $(word 2,$(_LIDAR_PORTS))
LIDAR_FRONT_RIGHT_ATTACHED ?= $(if $(filter-out -,$(HOST_LIDAR_FRONT_RIGHT_PORT)),1,0)
LIDAR_REAR_LEFT_ATTACHED   ?= $(if $(filter-out -,$(HOST_LIDAR_REAR_LEFT_PORT)),1,0)
MICRO_ROS_ATTACHED ?= $(call _HOST_DEVICE_EXISTS,HOST_MICRO_ROS_PORT)
ARM_ATTACHED       ?= $(call _HOST_DEVICE_EXISTS,HOST_ARM_PORT)
LEADER_ARM_ATTACHED ?= $(call _HOST_DEVICE_EXISTS,HOST_LEADER_ARM_PORT)
# HP60CとGemini 335Lは機体ごとに変わるパスが無いため、USBのベンダー/プロダクトIDで判定する。
HP60C_ATTACHED     ?= $(if $(filter 1,$(ROBOT_ATTACHED)),$(shell lsusb -d 3482:6723 >/dev/null 2>&1 && echo 1 || echo 0),0)
GEMINI_ATTACHED    ?= $(if $(filter 1,$(ROBOT_ATTACHED)),$(shell lsusb -d 2bc5:0804 >/dev/null 2>&1 && echo 1 || echo 0),0)
LIDAR_FRONT_RIGHT_COMPOSE_FILE := $(if $(filter 1,$(LIDAR_FRONT_RIGHT_ATTACHED)),edge/docker/docker-compose.lidar-front-right-attached.yml,)
LIDAR_REAR_LEFT_COMPOSE_FILE   := $(if $(filter 1,$(LIDAR_REAR_LEFT_ATTACHED)),edge/docker/docker-compose.lidar-rear-left-attached.yml,)
MICRO_ROS_COMPOSE_FILE := $(if $(filter 1,$(MICRO_ROS_ATTACHED)),edge/docker/docker-compose.micro-ros-attached.yml,)
ARM_COMPOSE_FILE       := $(if $(filter 1,$(ARM_ATTACHED)),edge/docker/docker-compose.arm-attached.yml,)
LEADER_ARM_COMPOSE_FILE := $(if $(filter 1,$(LEADER_ARM_ATTACHED)),edge/docker/docker-compose.leader-arm-attached.yml,)
HP60C_COMPOSE_FILE     := $(if $(filter 1,$(HP60C_ATTACHED)),edge/docker/docker-compose.hp60c-attached.yml,)
GEMINI_COMPOSE_FILE    := $(if $(filter 1,$(GEMINI_ATTACHED)),edge/docker/docker-compose.gemini-attached.yml,)
BASE_COMPOSE_FILE      := $(if $(filter 1,$(LIDAR_FRONT_RIGHT_ATTACHED) $(LIDAR_REAR_LEFT_ATTACHED) $(MICRO_ROS_ATTACHED) $(ARM_ATTACHED) $(HP60C_ATTACHED) $(GEMINI_ATTACHED)),edge/docker/docker-compose.base.yml,)

# 物体検出等のサーバー側処理用コンテナ(NVIDIA GPU前提)。ラズパイでは動かさない。
ifeq ($(UNAME_M),$(filter $(UNAME_M),aarch64 arm64))
PERCEPTION_COMPOSE_FILE :=
else
PERCEPTION_COMPOSE_FILE := edge/docker/docker-compose.perception.yml
endif

COMPOSE            := docker compose -f edge/docker/docker-compose.yml $(if $(GPU_COMPOSE_FILE),-f $(GPU_COMPOSE_FILE)) $(if $(PERCEPTION_COMPOSE_FILE),-f $(PERCEPTION_COMPOSE_FILE)) $(if $(LIDAR_FRONT_RIGHT_COMPOSE_FILE),-f $(LIDAR_FRONT_RIGHT_COMPOSE_FILE)) $(if $(LIDAR_REAR_LEFT_COMPOSE_FILE),-f $(LIDAR_REAR_LEFT_COMPOSE_FILE)) $(if $(MICRO_ROS_COMPOSE_FILE),-f $(MICRO_ROS_COMPOSE_FILE)) $(if $(ARM_COMPOSE_FILE),-f $(ARM_COMPOSE_FILE)) $(if $(LEADER_ARM_COMPOSE_FILE),-f $(LEADER_ARM_COMPOSE_FILE)) $(if $(HP60C_COMPOSE_FILE),-f $(HP60C_COMPOSE_FILE)) $(if $(GEMINI_COMPOSE_FILE),-f $(GEMINI_COMPOSE_FILE)) $(if $(BASE_COMPOSE_FILE),-f $(BASE_COMPOSE_FILE)) -p $(COMPOSE_PJ_NAME)
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
SERVER_PKGS        := amr_perception amr_moveit_config amr_manipulation amr_behaviors amr_robot_tools amr_teleop
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
	@test -f $(EDGE_ENV) || cp $(EDGE_ENV).example $(EDGE_ENV)
	$(COMPOSE) --env-file $(EDGE_ENV) up -d
	$(MAKE) install-packages

up-debug:
	@test -f $(EDGE_ENV) || cp $(EDGE_ENV).example $(EDGE_ENV)
	$(COMPOSE) --env-file $(EDGE_ENV) up
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
	@test -f $(EDGE_ENV) || cp $(EDGE_ENV).example $(EDGE_ENV)
	$(COMPOSE) --env-file $(EDGE_ENV) up -d

down:
	$(COMPOSE) --env-file $(EDGE_ENV) down

build:
	$(COMPOSE) --env-file $(EDGE_ENV) build

login:
	$(EXEC) $(ROS2_SERVICE) bash

# サーバー側の処理(点群生成・物体検出)を起動する。ノード本体はdiegobotコンテナでcolcon buildしたものを使う。
# object_resolver(名前で指定された物をVLMに選ばせる)が使うANTHROPIC_API_KEYは、起動のたびにedge/.envの値を渡す。
# 見分けた物を覚えておくデータベース(server/object_db)も立ち上げ、その接続先を渡す。
# DETECTOR=1で全フレームを推論し続けるobject_detectorも起動し、MODELでそのモデルを差し替えられる
# (例: make perception DETECTOR=1 MODEL=yoloe-26s-seg-pf.engine)。
DETECTOR ?=
MODEL ?=
perception:
	@. ./$(EDGE_ENV) && export OBJECT_DB_PASSWORD && docker compose -f server/object_db/docker-compose.yml up -d --quiet-pull
	@. ./$(EDGE_ENV) && $(EXEC) -e ANTHROPIC_API_KEY="$$ANTHROPIC_API_KEY" \
	  -e OBJECT_DB_URL="postgresql://diegobot:$$OBJECT_DB_PASSWORD@127.0.0.1:5433/objects" perception bash -c "source $(ROS_INSTALL_PREFIX)/setup.bash && source $(ROS2_WS)/install/local_setup.bash && ros2 launch amr_perception perception.launch.py $(if $(filter 1,$(DETECTOR)),detector:=true) $(if $(MODEL),model:=$(MODEL))"

# 実機のアームを開発機のMoveItで計画し、カメラで見つけた物をつかむ・置くアクション(/manipulator/pick, /manipulator/place)を出す。
# 物を探すため、先にmake perceptionも起動しておく。
manipulation:
	$(EXEC) $(ROS2_SERVICE) bash -c "$(CMD_ROS2_SOURCE) && $(CMD_ROS2_WS_SOURCE) && ros2 launch amr_manipulation manipulation.launch.py $(if $(RVIZ),use_rviz:=true)"

# 開発機に挿したSO-101のリーダーアームで実機のアームを動かす。Ctrl+CでMoveIt用のコントローラに戻す。
# 先にcheck_armで、止まっている実機のハードウェアとコントローラを動く状態に戻す。
teleop:
	$(EXEC) $(ROS2_SERVICE) bash -c "$(CMD_ROS2_SOURCE) && $(CMD_ROS2_WS_SOURCE) && ros2 run amr_teleop check_arm && ros2 launch amr_teleop leader_teleop.launch.py"

# テレオペをエピソードごとに/data/vla/raw/<DATASET>へ記録する(Enterで開始/保存)。TASKは学習に使う指示文。
# 記録を始める前に頭部をHEAD_POSE(パン,チルト[rad])に動かす。HEAD_POSE=を空にすると、その時点の向き(make adjust-head-cameraで合わせた向き)のまま使う。
DATASET   ?= test
TASK      ?=
HEAD_POSE ?= 0.0,-0.86
record-vla:
	$(if $(TASK),,$(error TASK に学習に使う指示文を渡してください(例: make record-vla TASK="Pick up the block")))
	$(EXEC) $(ROS2_SERVICE) bash -c "$(CMD_ROS2_SOURCE) && $(CMD_ROS2_WS_SOURCE) && ros2 run amr_teleop record_episodes --ros-args -p dataset:=$(DATASET) -p task:='$(TASK)' $(if $(HEAD_POSE),-p head_pose:=[$(HEAD_POSE)])"

# 実機の頭部(パン・チルト)をスライダーで動かす。
adjust-head-camera:
	$(MAKE) colcon CMD_RUN="ros2 run amr_teleop head_slider"

# 学習したポリシーで実機のアームを動かす(Enterで開始/停止)。LeRobot(torch)がROSのコンテナに無いため、
# 推論サーバーは開発機のPython(VLA_PYTHON)で動かし、コンテナのpolicy_runnerがTCPで使う。
VLA_PYTHON ?= $(HOME)/.platformio/penv/bin/python
POLICY     ?= data/vla/train/act_pick_block/checkpoints/last/pretrained_model
# 推論の出し方を変える引数(例: POLICY_ARGS="--temporal-ensemble 0.01"、"--n-action-steps 10")。
POLICY_ARGS ?=
run-policy:
	@$(VLA_PYTHON) server/vla/policy_server.py $(POLICY) $(POLICY_ARGS) & server=$$!; \
	trap "kill $$server 2>/dev/null" EXIT INT TERM; \
	$(VLA_PYTHON) -c "import socket,sys,time; end=time.time()+120; ok=lambda: socket.socket().connect_ex(('127.0.0.1',5555))==0; \
	[time.sleep(1) for _ in iter(lambda: ok() or time.time()>end, True)]; sys.exit(0 if ok() else 'policy server did not start')" && \
	$(EXEC) $(ROS2_SERVICE) bash -c "$(CMD_ROS2_SOURCE) && $(CMD_ROS2_WS_SOURCE) && ros2 run amr_teleop check_arm && ros2 run amr_teleop policy_runner $(if $(TASK),--ros-args -p task:='$(TASK)')"

# manipulationに加え、探す・近づくアクション(/behavior_server/search, /behavior_server/approach)を出す。
# 車体が動くため、周りに物が無い床で使う。先にmake perceptionも起動しておく。
behaviors:
	$(EXEC) $(ROS2_SERVICE) bash -c "$(CMD_ROS2_SOURCE) && $(CMD_ROS2_WS_SOURCE) && ros2 launch amr_behaviors behaviors.launch.py $(if $(RVIZ),use_rviz:=true)"

# Dify(LLM)から探す・近づく・つかむ・置くを呼ぶHTTP API(/openapi.jsonがスキーマ)。make perceptionとmake behaviorsも起動しておく。
# edge/.envはコンテナを作るときにしか読まれないため、起動のたびにその時点の値を渡す。
robot-tools:
	@. ./$(EDGE_ENV) && $(EXEC) -e ROBOT_TOOLS_PUBLIC_URL="$$ROBOT_TOOLS_PUBLIC_URL" -e ROBOT_TOOLS_API_KEY="$$ROBOT_TOOLS_API_KEY" \
	  $(ROS2_SERVICE) bash -c "$(CMD_ROS2_SOURCE) && $(CMD_ROS2_WS_SOURCE) && ros2 run amr_robot_tools robot_tools"

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

rqt-graph:
	$(MAKE) colcon CMD_RUN="rqt_graph"
# 頭部と手首のカラー画像だけを見る。view-robotは深度も購読し、ラズパイに深度の圧縮と送信をさせてしまう。
HEAD_IMAGE_TOPIC  ?= /head_camera/color/image_raw/compressed
WRIST_IMAGE_TOPIC ?= /camera/camera/color/image_raw/compressed
view-cameras:
	$(MAKE) colcon CMD_RUN="{ ros2 run rqt_image_view rqt_image_view $(HEAD_IMAGE_TOPIC) & ros2 run rqt_image_view rqt_image_view $(WRIST_IMAGE_TOPIC); wait; }"
# ロボットのモデルとTFを、スライダーで関節を動かしながらRVizで見る。偽の/joint_statesと/tfを出すため、
# 実機の通信に混ざらないよう別のROS_DOMAIN_IDで起動する。URDF=<urdf/内のファイル名>で部品だけも見られる。
URDF_VIEW_DOMAIN_ID ?= 99
URDF ?= amr.urdf.xacro
view-urdf:
	$(EXEC) -e ROS_DOMAIN_ID=$(URDF_VIEW_DOMAIN_ID) $(ROS2_SERVICE) bash -c "$(CMD_ROS2_SOURCE) && $(CMD_ROS2_WS_SOURCE) && ros2 launch amr_description display.launch.xml model:=$(URDF)"

tf-tree:
	$(MAKE) colcon CMD_RUN="ros2 run rqt_tf_tree rqt_tf_tree"
# 実機はTwistの/cmd_velで走る。SIM=1のときはGazeboのdiff_drive_controllerへTwistStampedで送る。
teleop-twist-keyboard:
	$(MAKE) colcon CMD_RUN="ros2 run teleop_twist_keyboard teleop_twist_keyboard $(if $(SIM),--ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel -p stamped:=true)"
PI_HOST ?= diegobot.local
PI_USER ?= daigo
pi-rsync:
	rsync -avz \
	  --exclude=edge/workspace/build/ \
	  --exclude=edge/workspace/install/ \
	  --exclude=edge/workspace/log/ \
	  --exclude=edge/workspace/build-arm64/ \
	  --exclude=server/ \
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
# 手首カメラ(HP60C)はUSBが一度抜けると、挿し直ってもドライバがつなぎ直さないため、コンテナごと起動し直す。
pi-restart-wrist-camera:
	ssh $(PI_USER)@$(PI_HOST) docker restart diegobot_hp60c
# 初回のラズパイにはmakeが無いことがあるため、ラズパイ上ではなく開発機からssh越しに実行する。
# -tはsudoのパスワード入力用。
pi-setup:
	ssh -t $(PI_USER)@$(PI_HOST) 'cd ~/diegobot/edge/provisioning && sudo ./setup.sh'
pi-setup-dev:
	ssh -t $(PI_USER)@$(PI_HOST) 'cd ~/diegobot/edge/provisioning && sudo ./setup.dev.sh'
# ESP32のファームを開発機でビルドし、ラズパイに挿したままesptoolで書き込む。
# ポートはmicro-ROS agentが開いているため、書き込む間だけagentのコンテナを止める。
# 2回目以降はアプリ本体だけ書く。ブートローダー・パーティション表も書き直すときはFULL=1。
PIO ?= $(HOME)/.platformio/penv/bin/pio
ESP32_BUILD := edge/esp32/.pio/build/esp32dev
ESP32_FLASH_DIR := /tmp/esp32-flash
pi-flash-esp32:
	cd edge/esp32 && $(PIO) run
	ssh $(PI_SSH_OPTS) $(PI_USER)@$(PI_HOST) 'mkdir -p $(ESP32_FLASH_DIR) && \
	  { [ -x ~/.venvs/esptool/bin/esptool.py ] || { python3 -m venv ~/.venvs/esptool && ~/.venvs/esptool/bin/pip install -q "esptool>=4.11,<5"; }; }'
	scp $(PI_SSH_OPTS) $(ESP32_BUILD)/firmware.bin $(if $(filter 1,$(FULL)),$(ESP32_BUILD)/bootloader.bin $(ESP32_BUILD)/partitions.bin $(HOME)/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin) \
	  $(PI_USER)@$(PI_HOST):$(ESP32_FLASH_DIR)/
	ssh $(PI_SSH_OPTS) $(PI_USER)@$(PI_HOST) 'set -e; . ~/diegobot/edge/.env; cd $(ESP32_FLASH_DIR); \
	  trap "docker start diegobot_micro_ros_agent >/dev/null" EXIT; \
	  docker stop diegobot_micro_ros_agent >/dev/null; \
	  ~/.venvs/esptool/bin/esptool.py --chip esp32 --port "$$HOST_MICRO_ROS_PORT" --baud 460800 write_flash -z \
	    --flash_mode dio --flash_freq 40m --flash_size detect \
	    $(if $(filter 1,$(FULL)),0x1000 bootloader.bin 0x8000 partitions.bin 0xe000 boot_app0.bin) 0x10000 firmware.bin'

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

# サーバー機で動かすDify(LLMアプリ基盤)。公式リポジトリのdocker/だけを版を固定して取得し、設定はserver/dify/.envに置く。
DIFY_VERSION := 1.18.0
DIFY_DIR     := server/dify
DIFY_DOCKER  := $(DIFY_DIR)/vendor/docker
DIFY_COMPOSE := docker compose --project-directory $(DIFY_DOCKER) -f $(DIFY_DOCKER)/docker-compose.yaml --env-file $(DIFY_DOCKER)/.env -p dify
dify-fetch:
	@if [ ! -d $(DIFY_DIR)/vendor/.git ]; then \
	  git clone --depth 1 --branch $(DIFY_VERSION) --filter=blob:none --sparse https://github.com/langgenius/dify.git $(DIFY_DIR)/vendor && \
	  git -C $(DIFY_DIR)/vendor sparse-checkout set docker; \
	elif [ "$$(git -C $(DIFY_DIR)/vendor describe --tags)" != "$(DIFY_VERSION)" ]; then \
	  git -C $(DIFY_DIR)/vendor fetch --depth 1 origin tag $(DIFY_VERSION) && git -C $(DIFY_DIR)/vendor checkout $(DIFY_VERSION); \
	fi
# server/dify/.envには変える値だけを置く。local_sandboxとagent_backendは認証トークンが空だと起動しない
# (公式の見本では空)ため、無ければ作って足す。
# DB_HOST等は渡さないとlocalhostになるため、公式の見本(起動に要る項目の一覧)の同じ名前の行をserver/dify/.envの値で
# 置き換えたものを、公式のcomposeが各サービスのenv_fileで読むvendor/docker/.envとして起動のたびに作る。
dify-up: dify-fetch
	@test -f $(DIFY_DIR)/.env || cp $(DIFY_DIR)/.env.example $(DIFY_DIR)/.env
	@grep -q '^DIFY_AGENT_LOCAL_SANDBOX_AUTH_TOKEN=.' $(DIFY_DIR)/.env || \
	  echo "DIFY_AGENT_LOCAL_SANDBOX_AUTH_TOKEN=$$(openssl rand -hex 32)" >> $(DIFY_DIR)/.env
	@test -f $(DIFY_DOCKER)/envs/core-services/shared.env || \
	  cp $(DIFY_DOCKER)/envs/core-services/shared.env.example $(DIFY_DOCKER)/envs/core-services/shared.env
	@rm -f $(DIFY_DOCKER)/.env
	@awk ' \
	  NR == FNR { if ($$0 ~ /^[A-Za-z_][A-Za-z0-9_]*=/) { k = substr($$0, 1, index($$0, "=") - 1); o[k] = $$0 } next } \
	  { k = substr($$0, 1, index($$0, "=") - 1); if (index($$0, "=") > 1 && k in o) { print o[k]; delete o[k] } else print } \
	  END { for (k in o) print o[k] }' $(DIFY_DIR)/.env $(DIFY_DOCKER)/.env.example > $(DIFY_DOCKER)/.env
	$(DIFY_COMPOSE) up -d
dify-down:
	$(DIFY_COMPOSE) down

# flutterはsnap経由でインストールされ/snap/binに入るが、呼び出し元シェルのPATHに
# /snap/binが無いと"flutter: not found"になるため、ここで明示的に足す。
android-install:
	cd $(MOBILE_DIR) && PATH="$$PATH:/snap/bin" flutter build apk --release && PATH="$$PATH:/snap/bin" flutter install
