#!/usr/bin/env bash
# edge/provisioning/setup.sh
#
# 量産を見据え、まっさらなラズパイを diegobot が動く状態まで持っていく手順を
# できるだけこの1本に集約する。Docker導入、AMR BLE Wi-Fiプロビジョニングデーモンの
# インストール、systemdサービスとしての常駐化を行う。
#
# 使い方:
#   cd ~/diegobot/edge/provisioning
#   sudo ./setup.sh
#
# 何度実行しても同じ結果になる（venv再作成・サービス再登録）ように作っている。

set -euo pipefail

# --- 設定値。環境に合わせて必要ならここだけ書き換える ---
SERVICE_USER="root"          # サービスを実行するOSユーザー。rootにしたい場合は "root" に変更
SERVICE_NAME="amr-provisioning.service"

# --- 事前チェック ---
if [[ ${EUID} -ne 0 ]]; then
  echo "エラー: このスクリプトは sudo で実行してください。" >&2
  echo "  例: sudo ./setup.sh" >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${SCRIPT_DIR}/venv"
SERVICE_TEMPLATE="${SCRIPT_DIR}/${SERVICE_NAME}.template"
SERVICE_DEST="/etc/systemd/system/${SERVICE_NAME}"

echo "==> [1/7] Dockerをインストール（未インストール時のみ）"
if ! command -v docker &>/dev/null; then
  curl -fsSL https://get.docker.com | sh
  systemctl enable --now docker
else
  echo "    docker は既にインストール済みのためスキップ"
fi

# sudoで叩いたログインユーザー自身が、後でmake up等をsudo無しで実行できるようにする
# (SERVICE_USERはBLEデーモン用の別ユーザーであり、ここでのdockerグループ対象とは無関係)。
INVOKING_USER="${SUDO_USER:-}"
if [[ -n "${INVOKING_USER}" ]] && id "${INVOKING_USER}" &>/dev/null; then
  usermod -aG docker "${INVOKING_USER}"
fi

echo "==> [2/7] APT依存関係をインストール"
# PyGObject/dbus-pythonはCPython拡張でpip installがソースビルドになるため、
# ビルドに必要なヘッダ類が無いと venv 内の pip install がそのまま失敗する。
apt-get update
apt-get install -y --no-install-recommends \
  python3-venv python3-dev pkg-config \
  libgirepository1.0-dev libcairo2-dev libdbus-1-dev libdbus-glib-1-dev

echo "==> [3/7] Python仮想環境を作成 (${VENV_DIR})"
python3 -m venv "${VENV_DIR}"
"${VENV_DIR}/bin/pip" install --upgrade pip
"${VENV_DIR}/bin/pip" install -r "${SCRIPT_DIR}/requirements.txt"

echo "==> [4/7] ${SERVICE_USER} を bluetooth グループへ追加"
if id "${SERVICE_USER}" &>/dev/null; then
  usermod -aG bluetooth "${SERVICE_USER}"
else
  echo "警告: ユーザー ${SERVICE_USER} が存在しません。SERVICE_USER の設定を確認してください。" >&2
fi

echo "==> [5/7] systemdサービスファイルを生成"
if [[ ! -f "${SERVICE_TEMPLATE}" ]]; then
  echo "エラー: テンプレートが見つかりません: ${SERVICE_TEMPLATE}" >&2
  exit 1
fi

sed \
  -e "s|__EXEC_START__|${VENV_DIR}/bin/python3 ${SCRIPT_DIR}/gatt_server.py|g" \
  -e "s|__WORKDIR__|${SCRIPT_DIR}|g" \
  -e "s|__SERVICE_USER__|${SERVICE_USER}|g" \
  "${SERVICE_TEMPLATE}" > "${SERVICE_DEST}"

echo "==> [6/7] systemdへ登録・有効化"
systemctl daemon-reload
systemctl enable --now "${SERVICE_NAME}"

echo "==> [7/7] 完了。以下で状態を確認できます:"
echo "    sudo systemctl status ${SERVICE_NAME}"
echo "    journalctl -u ${SERVICE_NAME} -f"
if [[ -n "${INVOKING_USER}" ]]; then
  echo "    (dockerグループの反映には ${INVOKING_USER} の再ログインが必要です)"
fi
