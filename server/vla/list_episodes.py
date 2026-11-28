"""record_episodesで記録したデータセット・エピソードの一覧を出す(make vla-listから呼ぶ)。

    python list_episodes.py data/vla/raw              # データセットごとの本数・記録した期間・指示文
    python list_episodes.py data/vla/raw <dataset>    # そのデータセットのエピソードごとの日時・長さ・物・メモ
"""
import argparse
import json
import re
import unicodedata
from datetime import datetime
from pathlib import Path

IMAGES = ('start_head.jpg', 'object_head.jpg', 'object_crop.jpg')


def episode_info(path):
    info_path = path / 'episode.json'
    info = json.loads(info_path.read_text()) if info_path.exists() else None
    recorded = None
    if info and info.get('recorded_at'):
        recorded = datetime.fromisoformat(info['recorded_at']).astimezone()
    else:
        # 日時を残すようにする前の記録は、bagの最初のメッセージを開発機で受け取った時刻を使う。
        meta = path / 'metadata.yaml'
        found = re.search(r'starting_time:\s*nanoseconds_since_epoch:\s*(\d+)', meta.read_text()) if meta.exists() else None
        if found:
            recorded = datetime.fromtimestamp(int(found.group(1)) / 1e9).astimezone()
    return info, recorded


def pad(text, width, right=False):
    """端末で全角が2文字分になるのに合わせて空白で揃える。"""
    text = str(text)
    space = ' ' * max(0, width - sum(2 if unicodedata.east_asian_width(c) in 'WF' else 1 for c in text))
    return space + text if right else text + space


def episodes(dataset):
    return sorted(d for d in dataset.glob('episode_*') if d.is_dir())


def when(t):
    return t.strftime('%Y-%m-%d %H:%M') if t else '?'


def list_datasets(root):
    names = sorted(d.name for d in root.iterdir() if d.is_dir() and not d.name.endswith('_failed'))
    print(f'{pad("データセット", 22)}{pad("成功", 10, True)}{pad("失敗", 6, True)}  {pad("記録した期間", 37)}'
          '指示文 / 測る物 / 場面のメモ')
    for name in names:
        rows = [episode_info(d) for d in episodes(root / name)]
        saved = [(i, t) for i, t in rows if i]
        failed = len([d for d in episodes(root / f'{name}_failed') if (d / 'episode.json').exists()])
        times = sorted(t for _, t in rows if t)
        period = f'{when(times[0])} 〜 {when(times[-1])}' if times else '?'
        latest = saved[-1][0] if saved else {}
        notes = [latest.get('task', '')]
        if (latest.get('object') or {}).get('queries'):
            notes.append(','.join(latest['object']['queries']))
        if latest.get('scene'):
            notes.append(latest['scene'])
        unsaved = len(rows) - len(saved)
        count = f'{len(saved)}' + (f'(+{unsaved}途中)' if unsaved else '')
        print(f'{pad(name, 22)}{pad(count, 10, True)}{pad(failed, 6, True)}  {pad(period, 37)}{" / ".join(notes)}')


def list_episodes(root, name):
    print(f'{pad("番号", 4, True)}  {pad("記録した日時", 18)}{pad("長さ", 7, True)}  {pad("結果", 6)}{pad("画像", 6)}'
          '物(確信度) / 寸法cm / 場面のメモ')
    for dataset, result in ((root / name, '成功'), (root / f'{name}_failed', '失敗')):
        for d in episodes(dataset):
            info, recorded = episode_info(d)
            index = int(d.name.split('_')[1])
            if info is None:
                print(f'{index:>4}  {pad(when(recorded), 18)}{"":>7}  保存されていない(記録の途中で終わった)')
                continue
            objects = (info.get('object') or {}).get('objects') or []
            best = max(objects, key=lambda o: o['score'], default=None)
            notes = [f'{best["label"]}({best["score"]:.2f})' if best else
                     ('見つからない' if info.get('object') and 'objects' in info['object'] else '-')]
            if info.get('object_size_cm'):
                notes.append('×'.join(f'{v:g}' for v in info['object_size_cm']))
            if info.get('scene'):
                notes.append(info['scene'])
            images = 'あり' if any((d / f).exists() for f in IMAGES) else '-'
            length = f'{info.get("duration", 0):.1f}秒'
            print(f'{index:>4}  {pad(when(recorded), 18)}{pad(length, 7, True)}  {pad(result, 6)}{pad(images, 6)}'
                  f'{" / ".join(notes)}')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('root', type=Path)
    parser.add_argument('dataset', nargs='?')
    args = parser.parse_args()
    if args.dataset:
        if not (args.root / args.dataset).is_dir() and not (args.root / f'{args.dataset}_failed').is_dir():
            raise SystemExit(f'{args.root / args.dataset} が無い')
        list_episodes(args.root, args.dataset)
    else:
        list_datasets(args.root)


if __name__ == '__main__':
    main()
