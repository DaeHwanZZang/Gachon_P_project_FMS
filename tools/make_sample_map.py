"""
샘플 맵 생성기
==============

뷰어와 로봇 클라이언트를 테스트할 창고형 occupancy grid 를 만든다.
실제 맵이 준비되기 전까지 쓰는 개발용 도구다.

    python tools/make_sample_map.py                 # maps/warehouse/warehouse.png + .json 생성
    python tools/make_sample_map.py --name test --width-m 10 --height-m 10

맵 하나 = maps/ 아래 폴더 하나 (예: maps/641931de9eae7cecb34d5765/ 참고).
--out-dir 를 생략하면 maps/<name>/ 을 자동으로 만든다.

색 규칙 (common/map_model.py 와 동일)
    255 흰색  = FREE      주행 가능
    128 회색  = UNKNOWN   미탐사 (주행 불가로 취급)
      0 검은색 = OCCUPIED  장애물
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from common.map_model import MapMetadata  # noqa: E402

FREE = 255
UNKNOWN = 128
OCCUPIED = 0


def build_warehouse(width_px: int, height_px: int, resolution: float) -> np.ndarray:
    """창고 레이아웃 하나를 그린다. 반환값은 grid[py][px] 그레이스케일 배열."""
    grid = np.full((height_px, width_px), FREE, dtype=np.uint8)

    def m2px(meters: float) -> int:
        return max(1, int(round(meters / resolution)))

    # 외벽 0.3m
    wall = m2px(0.3)
    grid[:wall, :] = OCCUPIED
    grid[-wall:, :] = OCCUPIED
    grid[:, :wall] = OCCUPIED
    grid[:, -wall:] = OCCUPIED

    # 선반 블록: 세로 방향 랙을 통로를 두고 반복 배치
    shelf_w = m2px(1.2)     # 선반 폭
    aisle_w = m2px(1.8)     # 통로 폭
    margin_x = m2px(2.0)    # 좌우 여백 (로봇 회차 공간)
    margin_y = m2px(2.5)    # 상하 여백 (메인 통로)

    x = wall + margin_x
    while x + shelf_w < width_px - wall - margin_x:
        grid[wall + margin_y : height_px - wall - margin_y, x : x + shelf_w] = OCCUPIED
        x += shelf_w + aisle_w

    # 미탐사 구역: 우상단 모서리는 아직 매핑되지 않은 것으로 둔다
    uh, uw = m2px(2.0), m2px(3.0)
    grid[wall : wall + uh, width_px - wall - uw : width_px - wall] = UNKNOWN

    # 충전 스테이션 근처 장애물 (기둥)
    pillar = m2px(0.4)
    cy = height_px - wall - m2px(1.2)
    for px_center_m in (3.0, 9.0, 15.0):
        cx = wall + m2px(px_center_m)
        if cx + pillar < width_px - wall:
            grid[cy : cy + pillar, cx : cx + pillar] = OCCUPIED

    return grid


def main() -> None:
    parser = argparse.ArgumentParser(description="샘플 occupancy grid 맵 생성")
    parser.add_argument("--name", default="warehouse", help="맵 이름 (파일명에도 사용)")
    parser.add_argument("--width-m", type=float, default=20.0, help="맵 가로 길이 (m)")
    parser.add_argument("--height-m", type=float, default=12.0, help="맵 세로 길이 (m)")
    parser.add_argument("--resolution", type=float, default=0.05, help="픽셀당 미터 (m/px)")
    parser.add_argument("--origin-x", type=float, default=None,
                        help="좌상단 world x (m). 생략하면 맵 정중앙이 (0,0) 이 되도록 자동 계산")
    parser.add_argument("--origin-y", type=float, default=None,
                        help="좌상단 world y (m). 생략하면 맵 정중앙이 (0,0) 이 되도록 자동 계산")
    parser.add_argument(
        "--out-dir", default=None,
        help="출력 디렉터리. 생략하면 maps/<name>/ (맵 하나 = 폴더 하나 원칙)",
    )
    args = parser.parse_args()

    width_px = int(round(args.width_m / args.resolution))
    height_px = int(round(args.height_m / args.resolution))

    origin_x = args.origin_x if args.origin_x is not None else -args.width_m / 2
    origin_y = args.origin_y if args.origin_y is not None else -args.height_m / 2

    grid = build_warehouse(width_px, height_px, args.resolution)

    out_dir = Path(args.out_dir) if args.out_dir else Path("maps") / args.name
    out_dir.mkdir(parents=True, exist_ok=True)
    image_name = f"{args.name}.png"
    Image.fromarray(grid, mode="L").save(out_dir / image_name)

    metadata = MapMetadata(
        name=args.name,
        image=image_name,
        resolution=args.resolution,
        origin_x=origin_x,
        origin_y=origin_y,
        width_px=width_px,
        height_px=height_px,
    )
    meta_path = out_dir / f"{args.name}.json"
    meta_path.write_text(metadata.model_dump_json(indent=2) + "\n", encoding="utf-8")

    free_ratio = float((grid == FREE).mean())
    print(f"생성: {out_dir / image_name}  ({width_px}x{height_px} px)")
    print(f"생성: {meta_path}")
    print(f"크기: {args.width_m} x {args.height_m} m @ {args.resolution} m/px")
    print(f"주행 가능 면적 비율: {free_ratio:.1%}")


if __name__ == "__main__":
    main()
