"""
벤더 맵 임포터 (grid_cfg.grid 계열)
===================================

산업 현장 AMR 벤더가 쓰는 occupancy grid 포맷을 **이미지 변환 없이** 그대로 쓰기
위한 도구다. `common/map_model.py` 의 좌표계(origin=좌상단, y 뒤집지 않음)를
바로 이 벤더 포맷에 맞춰뒀기 때문에, 여기서 하는 일은 딱 `grid_cfg.grid` 텍스트를
파싱해서 우리 스키마(`MapMetadata`)에 맞는 JSON 하나를 만드는 것뿐이다.
PNG 픽셀은 한 바이트도 안 건드린다.

    ox, oy           : 기준 픽셀의 world 좌표
    origin_px/py     : 그 기준 픽셀 위치
    scale_m2px       : 미터당 픽셀 수 (우리 resolution 필드는 역수, m/px)

변환 (world_to_pixel 이 뒤집지 않는 규약이므로 등식 하나로 끝난다):
    world(px, py) = world(origin_px, origin_py) + (px, py) 방향 그대로

    resolution = 1 / scale_m2px
    origin_x   = ox - origin_px * resolution     # pixel(0,0) 의 world x
    origin_y   = oy - origin_py * resolution     # pixel(0,0) 의 world y

사용:
    python tools/import_vendor_map.py maps/641931de9eae7cecb34d5765
    python tools/import_vendor_map.py maps/641931de9eae7cecb34d5765 --image edited_cleaning_gridmap.png

zone_meta.json(금지구역)/location_meta.json(스테이션)은 좌표계가 이미 우리와
동일(world m, 뒤집기 없음)이므로 변환 없이 그대로 참고할 수 있다 — 이 스크립트는
확인차 개수만 출력한다. FMS 쪽에서 실제로 쓰려면(금지구역을 장애물로 반영 등)
별도 작업이 필요하다.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from common.map_model import MapMetadata  # noqa: E402

DEFAULT_IMAGE = "edited_navi_gridmap.png"


def parse_grid_cfg(path: Path) -> dict[str, float]:
    values: dict[str, float] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or ":" not in line:
            continue
        key, _, raw = line.partition(":")
        values[key.strip()] = float(raw.strip())
    return values


def build_metadata(vendor_dir: Path, image: str, name: str | None,
                    occupied_thresh: int, free_thresh: int) -> MapMetadata:
    cfg = parse_grid_cfg(vendor_dir / "grid_cfg.grid")
    resolution = 1.0 / cfg["scale_m2px"]
    origin_x = cfg["ox"] - cfg["origin_px"] * resolution
    origin_y = cfg["oy"] - cfg["origin_py"] * resolution

    map_meta_path = vendor_dir / "map_meta.json"
    vendor_name = name
    if vendor_name is None and map_meta_path.exists():
        vendor_name = json.loads(map_meta_path.read_text(encoding="utf-8")).get("name")
    vendor_name = vendor_name or vendor_dir.name

    return MapMetadata(
        name=vendor_name,
        image=image,
        resolution=resolution,
        origin_x=origin_x,
        origin_y=origin_y,
        width_px=int(cfg["width_gm"]),
        height_px=int(cfg["height_gm"]),
        occupied_thresh=occupied_thresh,
        free_thresh=free_thresh,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="벤더 grid_cfg.grid 맵을 우리 MapMetadata JSON 으로 임포트")
    parser.add_argument("vendor_dir", help="grid_cfg.grid 가 있는 맵 폴더")
    parser.add_argument("--image", default=DEFAULT_IMAGE,
                        help=f"쓸 PNG 파일명 (기본 {DEFAULT_IMAGE} — 내비게이션용 수동편집본)")
    parser.add_argument("--name", default=None, help="맵 이름. 생략하면 map_meta.json 의 이름 사용")
    parser.add_argument("--out", default=None,
                        help="출력 JSON 경로. 생략하면 <vendor_dir>/fms_map.json")
    parser.add_argument("--occupied-thresh", type=int, default=64)
    parser.add_argument("--free-thresh", type=int, default=192)
    args = parser.parse_args()

    vendor_dir = Path(args.vendor_dir)
    if not (vendor_dir / args.image).exists():
        parser.error(f"{vendor_dir / args.image} 없음")

    metadata = build_metadata(vendor_dir, args.image, args.name,
                               args.occupied_thresh, args.free_thresh)

    out_path = Path(args.out) if args.out else vendor_dir / "fms_map.json"
    out_path.write_text(metadata.model_dump_json(indent=2) + "\n", encoding="utf-8")

    print(f"생성: {out_path}")
    print(f"  image={metadata.image}  resolution={metadata.resolution:.5f} m/px")
    print(f"  origin=({metadata.origin_x:.3f}, {metadata.origin_y:.3f})  "
          f"{metadata.width_m:.2f} x {metadata.height_m:.2f} m")

    zone_path = vendor_dir / "zone_meta.json"
    if zone_path.exists():
        zones = json.loads(zone_path.read_text(encoding="utf-8")).get("zones", [])
        print(f"  참고: zone_meta.json 에 구역 {len(zones)}개 "
              f"({', '.join(sorted({z['type'] for z in zones})) or '-'}) — 변환 안 됨, 필요시 별도 작업")

    loc_path = vendor_dir / "location_meta.json"
    if loc_path.exists():
        locs = json.loads(loc_path.read_text(encoding="utf-8")).get("locations", [])
        print(f"  참고: location_meta.json 에 지점 {len(locs)}개 — 변환 안 됨, 필요시 별도 작업")

    return 0


if __name__ == "__main__":
    sys.exit(main())
