"""
외부 맵 이미지 전처리
====================

라이다 SLAM 등 외부에서 만든 grid map 이미지를 이 프로젝트의 맵 포맷
(PNG + `MapMetadata` JSON, `maps/` 아래 한 쌍)으로 바꾼다.

이 스크립트가 하는 일은 딱 둘이다.
  1. 이미지를 그레이스케일로 저장해 `maps/` 에 놓는다 (픽셀 값 자체는 바꾸지 않는다 —
     장애물 팽창은 여기서 하지 않고 로봇 쪽 경로계획(`robot_client/planner.py`)이
     로봇 몸체 크기를 반영해서 알아서 한다)
  2. 가로 픽셀 수와 `--width-m` 로 resolution(m/px)을 계산해 메타데이터를 쓴다

threshold(`--occupied-thresh` / `--free-thresh`) 는 원본 이미지의 그레이 분포를
보고 맞춰야 한다. 예를 들어 라이다 맵이 흰색(주행 가능)/검은 선(벽)/연회색
(미탐사 배경) 세 값만 쓴다면, 연회색이 `free_thresh` 밑으로 떨어지게 잡아야
UNKNOWN 으로 분류된다 (기본 임계값 192 는 연회색 배경까지 FREE 로 오분류하기 쉽다).
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from PIL import Image

import sys

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from common.map_model import MapMetadata  # noqa: E402


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="외부 맵 이미지 -> maps/ 포맷 변환")
    p.add_argument("image", help="원본 맵 이미지 경로")
    p.add_argument("name", help="맵 이름. maps/<name>.png, maps/<name>.json 으로 저장")
    p.add_argument("--width-m", type=float, required=True, help="이미지 가로 전체가 실제 몇 미터인지")
    p.add_argument("--maps-dir", default="maps", help="출력 디렉터리 (기본 maps/)")
    p.add_argument("--occupied-thresh", type=int, default=64, help="이 값 이하 grey = OCCUPIED")
    p.add_argument("--free-thresh", type=int, default=240, help="이 값 이상 grey = FREE")
    p.add_argument("--origin-x", type=float, default=None,
                   help="이미지 좌상단 픽셀의 world x (m). 생략하면 맵 정중앙이 (0,0) 이 되도록 자동 계산")
    p.add_argument("--origin-y", type=float, default=None,
                   help="이미지 좌상단 픽셀의 world y (m). 생략하면 맵 정중앙이 (0,0) 이 되도록 자동 계산")
    return p


def main() -> int:
    args = build_arg_parser().parse_args()

    src = Path(args.image)
    img = Image.open(src).convert("L")
    width_px, height_px = img.size
    resolution = args.width_m / width_px
    height_m = height_px * resolution
    origin_x = args.origin_x if args.origin_x is not None else -args.width_m / 2
    origin_y = args.origin_y if args.origin_y is not None else -height_m / 2

    maps_dir = Path(args.maps_dir)
    maps_dir.mkdir(parents=True, exist_ok=True)
    out_image = f"{args.name}.png"
    img.save(maps_dir / out_image)

    metadata = MapMetadata(
        name=args.name,
        image=out_image,
        resolution=resolution,
        origin_x=origin_x,
        origin_y=origin_y,
        width_px=width_px,
        height_px=height_px,
        occupied_thresh=args.occupied_thresh,
        free_thresh=args.free_thresh,
    )
    out_json = maps_dir / f"{args.name}.json"
    out_json.write_text(metadata.model_dump_json(indent=2), encoding="utf-8")

    grey = np.asarray(img, dtype=np.uint8)
    n_occupied = int(np.sum(grey <= args.occupied_thresh))
    n_free = int(np.sum(grey >= args.free_thresh))
    n_unknown = grey.size - n_occupied - n_free

    print(f"{src} -> {maps_dir / out_image}, {out_json}")
    print(f"크기: {width_px} x {height_px} px, {metadata.width_m:.2f} x {metadata.height_m:.2f} m "
          f"(resolution {resolution:.5f} m/px)")
    print(f"셀 분류: FREE {n_free}px, UNKNOWN {n_unknown}px, OCCUPIED {n_occupied}px")
    if n_unknown / grey.size > 0.5:
        print("경고: UNKNOWN 비율이 절반 넘음 — threshold 가 안 맞을 수 있다")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
