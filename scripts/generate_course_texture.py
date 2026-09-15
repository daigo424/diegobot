#!/usr/bin/env python3
"""赤テープの周回コースを模したGazebo ground plane用テクスチャ画像を生成する。"""

import re
from pathlib import Path

from PIL import Image, ImageDraw

WORLD_SIZE_M = 6.0
IMAGE_SIZE_PX = 4096
PIXELS_PER_METER = IMAGE_SIZE_PX / WORLD_SIZE_M

TAPE_WIDTH_M = 0.05
TAPE_WIDTH_PX = round(TAPE_WIDTH_M * PIXELS_PER_METER)

TAPE_COLOR = (200, 30, 30)
BACKGROUND_COLOR = (255, 255, 255)

# 周回コースの境界(m単位、world原点(0,0)が画像中心)。外周は閉じた四角形、内周は1辺
# (北側)を開けた3辺だけの多角形にして、中心(0,0)からロボットが内周の切れ目を通って
# 通路に出られるようにする(閉じた内周だと中心が非走行エリアの中に閉じ込められるため)。
COURSE_LOOPS_M = {
    "outer": {"points": [(-2.6, -2.0), (2.6, -2.0), (2.6, 2.0), (-2.6, 2.0)], "closed": True},
    "inner": {"points": [(-1.0, 0.8), (-1.0, -0.8), (1.0, -0.8), (1.0, 0.8)], "closed": False},
}

OUTPUT_DIR = Path(__file__).resolve().parent.parent / "data" / "courses"
OUTPUT_NAME_RE = re.compile(r"^autogene-course(\d+)\.png$")


def world_to_pixel(x_m: float, y_m: float) -> tuple[int, int]:
    px = IMAGE_SIZE_PX / 2 + x_m * PIXELS_PER_METER
    py = IMAGE_SIZE_PX / 2 - y_m * PIXELS_PER_METER
    return round(px), round(py)


def next_output_path() -> Path:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    existing_numbers = [
        int(match.group(1))
        for f in OUTPUT_DIR.glob("autogene-course*.png")
        if (match := OUTPUT_NAME_RE.match(f.name))
    ]
    next_number = max(existing_numbers, default=0) + 1
    return OUTPUT_DIR / f"autogene-course{next_number}.png"


def generate() -> Path:
    image = Image.new("RGB", (IMAGE_SIZE_PX, IMAGE_SIZE_PX), BACKGROUND_COLOR)
    draw = ImageDraw.Draw(image)

    radius = TAPE_WIDTH_PX / 2
    for loop in COURSE_LOOPS_M.values():
        points_px = [world_to_pixel(x, y) for x, y in loop["points"]]
        line_points_px = points_px + [points_px[0]] if loop["closed"] else points_px
        draw.line(line_points_px, fill=TAPE_COLOR, width=TAPE_WIDTH_PX, joint="curve")

        # draw.lineのjoint="curve"だけだと角が欠けることがあるため、
        # 各頂点に同径の円を重ねてテープの継ぎ目を隙間なく見せる。
        for px, py in points_px:
            draw.ellipse([px - radius, py - radius, px + radius, py + radius], fill=TAPE_COLOR)

    output_path = next_output_path()
    image.save(output_path)
    return output_path


if __name__ == "__main__":
    path = generate()
    print(f"generated: {path}")
