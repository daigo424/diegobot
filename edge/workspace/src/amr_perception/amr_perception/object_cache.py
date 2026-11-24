"""VLMで見分けた物の見た目(埋め込み)と答えを覚えておき、2回目以降はVLMを呼ばずに当てる。"""
import os
import uuid

import cv2
import numpy as np


class Embedder:
    """切り出した画像を、同じ1個の物かを比べる埋め込みにする(DINOv2。画像だけで学習しているため、
    同じ物の別の画像どうしが近くなりやすい)。"""

    def __init__(self, model='dinov2_vits14', device=None, cache_dir=None):
        import torch  # ROSのテストなど、使わない環境では読み込まない
        self.torch = torch
        if cache_dir:
            torch.hub.set_dir(cache_dir)
        self.device = device or ('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = torch.hub.load('facebookresearch/dinov2', model).to(self.device).eval()
        self.name = model
        self.mean = np.array([0.485, 0.456, 0.406], np.float32)
        self.std = np.array([0.229, 0.224, 0.225], np.float32)

    def __call__(self, crops):
        """BGRの切り出しの並びから、長さ1に揃えた埋め込みの並びを返す。"""
        batch = []
        for crop in crops:
            rgb = cv2.cvtColor(cv2.resize(crop, (224, 224), interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2RGB)
            batch.append(((rgb.astype(np.float32) / 255.0 - self.mean) / self.std).transpose(2, 0, 1))
        with self.torch.no_grad():
            x = self.torch.from_numpy(np.stack(batch)).to(self.device)
            out = self.model(x).float().cpu().numpy()
        return out / np.linalg.norm(out, axis=1, keepdims=True)


class ObjectCache:
    """埋め込みと答え(名前ごとに、その物だったか)をPostgreSQL(pgvector)に置く。表はserver/object_db/init.sql。"""

    def __init__(self, url, model_name, image_dir):
        import psycopg  # データベースを使わない環境では読み込まない
        self.connection = psycopg.connect(url, autocommit=True)
        self.model_name = model_name
        self.image_dir = image_dir

    def lookup(self, name, embedding, limit=20):
        """nameについて、似た見た目の観測のうち一番近い「その物だった」「違った」の類似度(無ければ0)を返す。"""
        rows = self.connection.execute(
            'SELECT v.match, 1 - (o.embedding <=> %s::vector) FROM observations o '
            'JOIN verdicts v ON v.observation_id = o.id WHERE v.name = %s AND o.model = %s '
            'ORDER BY o.embedding <=> %s::vector LIMIT %s',
            (vector(embedding), name, self.model_name, vector(embedding), limit)).fetchall()
        positive = max((s for match, s in rows if match), default=0.0)
        negative = max((s for match, s in rows if not match), default=0.0)
        return positive, negative

    def remember(self, name, embedding, crop, crop_px, match, source='vlm'):
        path = None
        if self.image_dir:
            os.makedirs(self.image_dir, exist_ok=True)
            path = os.path.join(self.image_dir, f'{uuid.uuid4().hex}.jpg')
            cv2.imwrite(path, crop)
        observation = self.connection.execute(
            'INSERT INTO observations (model, embedding, image_path, crop_px) VALUES (%s, %s::vector, %s, %s) '
            'RETURNING id', (self.model_name, vector(embedding), path, float(crop_px))).fetchone()[0]
        self.connection.execute('INSERT INTO verdicts (observation_id, name, match, source) VALUES (%s, %s, %s, %s)',
                                (observation, name, bool(match), source))


def vector(values):
    return '[' + ','.join(f'{v:.6f}' for v in values) + ']'


def decide(scores, threshold, margin):
    """候補ごとの(その物だった類似度, 違った類似度)から、キャッシュだけで決められる答えを返す。
    ('match', 番号) / ('none', None) / (None, None)=決められない(VLMに聞く)。番号は0から。"""
    hits = [i for i, (pos, neg) in enumerate(scores) if pos >= threshold and pos - neg >= margin]
    if len(hits) == 1:
        return 'match', hits[0]
    if not hits and scores and all(neg >= threshold and neg - pos >= margin for pos, neg in scores):
        return 'none', None
    return None, None
