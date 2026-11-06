"""名前で指定された物を、object_finderの候補の中からVLMに選ばせて1つに決めるサービス(/object_resolver/resolve)。"""
import base64
import math
import os
import re
import threading

import cv2
import numpy as np
import rclpy
from amr_interfaces.srv import FindObjects, ResolveObject
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformException, TransformListener
import tf2_geometry_msgs  # noqa: F401  PointStampedをBuffer.transformで変換できるよう登録する

_HEAD = '番号を付けた枠に、物が1つずつ写っています(番号は枠の左上の赤い数字)。「{name}」はどれですか。'
_TAIL = ('最後の行に「答え: 番号」と書いてください。当てはまる物が無ければ「答え: 0」、'
         '写りが悪くて見分けられなければ「答え: -1」と書いてください。')
# 選び方の質問。VLMは当てはまる物が無くても何かを選びがちなため、特徴を確かめてから答えさせる型も用意し、比べて選ぶ。
PROMPTS = {
    'plain': _HEAD + '名前や説明の特徴がすべて合う物だけを選び、1つでも明らかに違う物は選ばないでください。' + _TAIL,
    'checklist': _HEAD + ('まず名前や説明を特徴ごとに分け、各番号の物について、特徴ごとに「合う」「合わない」「見えない」を'
                          '短く書いてください。すべての特徴が合う物だけを選び、1つでも合わない物は選ばないでください。') + _TAIL,
    'describe': ('番号を付けた枠に、物が1つずつ写っています(番号は枠の左上の赤い数字)。まず各番号の物の見た目'
                 '(何の物か、髪や体の色、服、持ち物など)を1行ずつ書いてください。次に、その説明と「{name}」を比べ、'
                 '特徴がすべて合う物だけを選んでください。1つでも合わない物は選ばないでください。') + _TAIL,
}
# 選んだ物を、最初の答えを見せずに別に確かめさせる(答えを見せると同じ誤りを繰り返しやすい)。
# 「はい」と答えやすい偏りを避けるため、合うかではなく特徴ごとに写っているかを答えさせる。
VERIFY_PROMPT = ('この画像の物について、「{name}」という名前や説明の特徴を1つずつ挙げ、それぞれ「写っている」'
                 '「写っていない」「分からない」で答えてください。最後の行に、すべて写っていれば「判定: 合う」、'
                 '1つでも写っていなければ「判定: 合わない」、分からない特徴があれば「判定: 不明」と書いてください。')


def crop_square(image, obj, margin):
    """外接矩形の周りを少し広げて正方形に切り出す(足りない所は白で埋める)。"""
    cx, cy = obj.bbox.center.position.x, obj.bbox.center.position.y
    half = max(obj.bbox.size_x, obj.bbox.size_y) * (1.0 + margin) / 2.0
    x0, y0, x1, y1 = (int(round(v)) for v in (cx - half, cy - half, cx + half, cy + half))
    out = np.full((y1 - y0, x1 - x0, 3), 255, np.uint8)
    h, w = image.shape[:2]
    sx0, sy0, sx1, sy1 = max(x0, 0), max(y0, 0), min(x1, w), min(y1, h)
    if sx1 > sx0 and sy1 > sy0:
        out[sy0 - y0:sy1 - y0, sx0 - x0:sx1 - x0] = image[sy0:sy1, sx0:sx1]
    return out


def make_grid(image, objects, tile_px, margin):
    """候補の切り出しを番号付きで並べた1枚の画像にする。番号は1から。"""
    tiles = []
    for i, obj in enumerate(objects, 1):
        # 拡大しても情報は増えないが、VLMの画像の分割が細かくなりすぎないよう、ある程度の大きさで渡す。
        tile = cv2.resize(crop_square(image, obj, margin), (tile_px, tile_px), interpolation=cv2.INTER_LINEAR)
        cv2.rectangle(tile, (0, 0), (tile_px - 1, tile_px - 1), (0, 0, 0), 2)
        cv2.putText(tile, str(i), (8, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (255, 255, 255), 6)
        cv2.putText(tile, str(i), (8, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 0, 255), 3)
        tiles.append(tile)
    cols = min(len(tiles), 4)
    rows = math.ceil(len(tiles) / cols)
    tiles += [np.full_like(tiles[0], 255)] * (rows * cols - len(tiles))
    return np.vstack([np.hstack(tiles[r * cols:(r + 1) * cols]) for r in range(rows)])


def parse_answer(text, count):
    """VLMの答え(最後の「答え: 番号」、無ければ数字だけの答え)から番号を取り出す。
    1〜countなら番号、0は該当なし、-1は見分けられない。読めなければNone。"""
    found = re.findall(r'答え\s*[:：]\s*(-?\d+)', text or '')
    if found:
        value = int(found[-1])
    elif re.fullmatch(r'\s*-?\d+\s*', text or ''):
        value = int(text)
    else:
        return None
    return value if -1 <= value <= count else None


def parse_verdict(text):
    """確かめの答えの最後の「判定: 合う/合わない/不明」を返す。読めなければNone。"""
    found = re.findall(r'判定\s*[:：]\s*(合わない|合う|不明)', text or '')
    return found[-1] if found else None


class AnthropicChooser:
    """番号付きの画像をClaudeに見せて番号を答えさせる。"""

    def __init__(self, model, timeout):
        import anthropic  # キーが無い環境やテストでは読み込まない
        self.anthropic = anthropic
        self.client = anthropic.Anthropic(timeout=timeout, max_retries=2)
        self.model = model

    def __call__(self, jpeg, text, effort='low'):
        response = self.client.beta.messages.create(
            model=self.model,
            max_tokens=4096,
            output_config={'effort': effort},
            # 安全の判定で断られたとき、サーバー側で別のモデルに回して答えを得る。
            betas=['server-side-fallback-2026-07-01'],
            fallbacks='default',
            messages=[{'role': 'user', 'content': [
                {'type': 'image', 'source': {'type': 'base64', 'media_type': 'image/jpeg',
                                             'data': base64.standard_b64encode(jpeg).decode()}},
                {'type': 'text', 'text': text},
            ]}],
        )
        if response.stop_reason == 'refusal':
            raise RuntimeError(f'the model declined: {response.stop_details}')
        return ''.join(block.text for block in response.content if block.type == 'text')


class ObjectResolver(Node):
    def __init__(self, chooser=None):
        super().__init__('object_resolver')
        p = self.declare_parameter
        # 評価のため、見分けに関わる値はサービスが呼ばれるたびにパラメータから読み直す(ros2 param setで変えられる)。
        self.default_queries = p('default_queries', ['figurine', 'toy figure', 'doll', 'stuffed toy',
                                                     'mascot figure']).value
        # 候補は取りこぼさないことを優先し、余分な物はVLMが落とす。
        self.min_score = p('min_score', 0.08).value
        self.max_candidates = p('max_candidates', 8).value
        # これより大きい物は対象(手で持てる大きさのフィギュア)ではないとみなす。
        self.max_size = p('max_size', 0.15).value
        # これより小さく写っている候補はVLMに見せない(見分けられないため)。
        self.min_crop_px = p('min_crop_px', 100.0).value
        self.exclude_radius = p('exclude_radius', 0.08).value
        self.use_color = p('use_color', False).value
        self.tile_px = p('tile_px', 336).value
        self.crop_margin = p('crop_margin', 0.15).value
        # VLMへの質問の型(PROMPTSのキー)と、選んだ物を別に確かめるか、VLMが考える量。
        self.prompt_style = p('prompt_style', 'plain').value
        self.verify = p('verify', False).value
        self.effort = p('effort', 'low').value
        self.find_timeout = p('find_timeout', 15.0).value
        self.odom_frame = p('odom_frame', 'odom').value
        model = p('model', 'claude-sonnet-5-5').value
        timeout = p('vlm_timeout', 30.0).value

        self.chooser = chooser
        if self.chooser is None and os.environ.get('ANTHROPIC_API_KEY'):
            self.chooser = AnthropicChooser(model, timeout)
        if self.chooser is None:
            self.get_logger().warn('ANTHROPIC_API_KEY is not set; resolve will fail until it is')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, None, spin_thread=True)
        # サービスの処理の中からobject_finderを呼んで待つため、応答を受け取るコールバックを別のグループにする
        # (同じグループだと、処理が終わるまで応答が受け取れず止まる)。
        self.find_client = self.create_client(FindObjects, '/object_finder/find',
                                              callback_group=MutuallyExclusiveCallbackGroup())
        self.grid_pub = self.create_publisher(Image, '~/grid', 1)
        self.create_service(ResolveObject, '~/resolve', self.on_resolve,
                            callback_group=MutuallyExclusiveCallbackGroup())

    def on_resolve(self, request, response):
        try:
            return self.resolve(request, response)
        except Exception as e:  # VLMやネットワークの失敗も結果として返し、呼ぶ側を待たせない
            self.get_logger().error(f'resolve failed: {e}')
            response.error_code, response.message = ResolveObject.Response.FAILED, str(e)
            return response

    def resolve(self, request, response):
        for name in ('default_queries', 'min_score', 'max_candidates', 'max_size', 'min_crop_px', 'exclude_radius',
                     'use_color', 'tile_px', 'crop_margin', 'prompt_style', 'verify', 'effort'):
            setattr(self, name, self.get_parameter(name).value)
        name = request.target_name.strip()
        queries = [q for q in request.queries if q.strip()] or list(self.default_queries)
        color = request.color if self.use_color else ''
        found = self.find(queries, color)
        if found is None:
            response.error_code, response.message = ResolveObject.Response.FAILED, 'object_finder did not answer'
            return response
        if not found.success:
            response.error_code, response.message = ResolveObject.Response.FAILED, found.message
            return response
        candidates = self.select(found.objects, color, request.exclude)
        if not candidates:
            response.error_code = ResolveObject.Response.NOT_FOUND
            response.message = f'no candidate for "{name}"'
            return response
        visible = [o for o in candidates if o.crop_px >= self.min_crop_px]
        if not visible:
            response.error_code, response.candidates = ResolveObject.Response.TOO_FAR, candidates
            response.message = f'{len(candidates)} candidate(s), too small to identify'
            return response
        if self.chooser is None:
            response.error_code, response.message = ResolveObject.Response.FAILED, 'ANTHROPIC_API_KEY is not set'
            return response

        image = cv2.imdecode(np.frombuffer(found.image.data, np.uint8), cv2.IMREAD_COLOR)
        grid = make_grid(image, visible, self.tile_px, self.crop_margin)
        self.publish_grid(found.image.header, grid)
        ok, jpeg = cv2.imencode('.jpg', grid, [cv2.IMWRITE_JPEG_QUALITY, 90])
        answer = None
        for _ in range(2):
            text = self.chooser(jpeg.tobytes(), PROMPTS[self.prompt_style].format(name=name), self.effort)
            answer = parse_answer(text, len(visible))
            if answer is not None:
                break
            self.get_logger().warn(f'unexpected answer from the model: {text!r}')
        self.get_logger().info(f'"{name}" among {len(visible)} candidate(s): answer {answer}')
        verdict = None
        if self.verify and answer is not None and answer > 0:
            tile = make_grid(image, [visible[answer - 1]], self.tile_px, self.crop_margin)
            ok, single = cv2.imencode('.jpg', tile, [cv2.IMWRITE_JPEG_QUALITY, 90])
            verdict = parse_verdict(self.chooser(single.tobytes(), VERIFY_PROMPT.format(name=name), self.effort))
            self.get_logger().info(f'verification of candidate {answer}: {verdict}')
        others = [o for o in candidates if o.crop_px < self.min_crop_px]
        if answer is None:
            response.error_code, response.message = ResolveObject.Response.FAILED, 'the model gave no valid number'
        elif verdict in ('合わない', None) and self.verify and answer > 0:
            response.error_code, response.candidates = ResolveObject.Response.NO_MATCH, others
            response.message = f'candidate {answer} did not pass the check for "{name}" ({verdict})'
        elif verdict == '不明':
            response.error_code, response.message = ResolveObject.Response.AMBIGUOUS, 'the check could not tell'
        elif answer == -1:
            response.error_code, response.message = ResolveObject.Response.AMBIGUOUS, 'the model could not tell'
        elif answer == 0:
            # 見せた候補は対象ではないと分かったので、まだ確かめていない(小さく写った)候補だけを返す。
            response.error_code, response.candidates = ResolveObject.Response.NO_MATCH, others
            response.message = f'none of {len(visible)} candidate(s) is "{name}"'
        else:
            response.error_code, response.object = ResolveObject.Response.SUCCESS, visible[answer - 1]
            response.message = f'"{name}" is candidate {answer} of {len(visible)}'
        return response

    def find(self, queries, color):
        if not self.find_client.wait_for_service(timeout_sec=2.0):
            return None
        targets = [f'{color} {q}'.strip() for q in queries]
        done = threading.Event()
        future = self.find_client.call_async(
            FindObjects.Request(queries=targets, min_score=float(self.min_score), want_mask=True))
        future.add_done_callback(lambda _: done.set())
        return future.result() if done.wait(self.find_timeout) else None

    def select(self, objects, color, exclude):
        """位置が分かり、大きさが対象らしく、除外の位置から離れた候補を、確信度の高い順に上限まで残す。"""
        kept = []
        for obj in objects:
            if not obj.position_valid or (color and obj.color != color):
                continue
            if max(obj.width, obj.height) > self.max_size:
                continue
            if exclude and self.near_excluded(obj, exclude):
                continue
            kept.append(obj)
        kept.sort(key=lambda o: o.score, reverse=True)
        return kept[:self.max_candidates]

    def near_excluded(self, obj, exclude):
        try:
            p = self.tf_buffer.transform(obj.position, self.odom_frame, timeout=Duration(seconds=0.3)).point
        except TransformException as e:
            self.get_logger().warn(f'cannot transform to {self.odom_frame}: {e}')
            return False
        return any(math.hypot(p.x - e.x, p.y - e.y) <= self.exclude_radius for e in exclude)

    def publish_grid(self, header, grid):
        if self.grid_pub.get_subscription_count() > 0:
            self.grid_pub.publish(Image(header=header, height=grid.shape[0], width=grid.shape[1], encoding='bgr8',
                                        step=grid.shape[1] * 3, data=grid.tobytes()))


def main():
    rclpy.init()
    node = ObjectResolver()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
