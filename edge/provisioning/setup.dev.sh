#!/usr/bin/env bash
# edge/provisioning/setup.dev.sh
#
# setup.sh(量産機向けの必須セットアップ)に加えて、開発・検証用の
# 追加ツールを入れる。量産機には不要なため setup.sh 本体には含めない。
#
# 使い方:
#   cd ~/diegobot/edge/provisioning
#   sudo ./setup.dev.sh

set -euo pipefail

if [[ ${EUID} -ne 0 ]]; then
  echo "エラー: このスクリプトは sudo で実行してください。" >&2
  echo "  例: sudo ./setup.dev.sh" >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "==> [1/2] setup.sh を実行"
"${SCRIPT_DIR}/setup.sh"

echo "==> [2/2] 開発用ツールをインストール"
apt-get install -y --no-install-recommends vim

echo "==> 完了"
