#!/usr/bin/env python3
"""Search the local ET1 preset .trk library."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


DEFAULT_ROOT = Path(__file__).resolve().parents[1] / "assets"
MAGIC = b"ET1TRK1\x00"

CATEGORY_ALIASES = {
    "LDLE动作": ["idle", "standby", "rest", "空闲", "待机", "ldle", "idle action"],
    "talk": ["talk", "speak", "speaking", "chat", "说话", "讲话", "聊天", "口播"],
    "举手": ["raise hand", "raise arm", "hand up", "举手", "抬手"],
    "傲娇": ["proud", "tsundere", "傲娇", "不服"],
    "击掌": ["high five", "hi five", "击掌", "拍手"],
    "呐喊": ["shout", "yell", "call out", "呐喊", "呼喊", "喊"],
    "害怕": ["afraid", "scared", "fear", "frightened", "害怕", "恐惧"],
    "害羞": ["shy", "bashful", "embarrassed", "害羞", "羞涩"],
    "开心": ["happy", "joy", "joyful", "excited", "开心", "高兴", "兴奋"],
    "手臂比心": ["arm heart", "heart", "heart gesture", "love", "比心", "爱心", "手臂比心"],
    "挥手": ["wave", "waving", "greet", "hello", "挥手", "招手"],
    "握手": ["handshake", "shake hands", "握手"],
    "摆拍": ["pose", "photo pose", "posing", "摆拍", "拍照"],
    "敬礼": ["salute", "敬礼"],
    "武术": ["martial arts", "kung fu", "kungfu", "wushu", "武术", "功夫"],
    "绅士礼": ["bow", "gentleman bow", "gentleman", "绅士礼", "鞠躬", "行礼"],
    "飞吻": ["blow kiss", "kiss", "flying kiss", "飞吻", "亲吻"],
}


@dataclass(frozen=True)
class Preset:
    path: Path
    relpath: str
    category: str
    filename: str
    frames: int | None

    def to_dict(
        self,
        *,
        score: int | None = None,
        fps: float = 50.0,
        staged_path: Path | None = None,
    ) -> dict[str, object]:
        duration = None
        if self.frames is not None and fps > 0:
            duration = max(self.frames - 1, 0) / fps
        out: dict[str, object] = {
            "path": str(self.path),
            "relpath": self.relpath,
            "category": self.category,
            "filename": self.filename,
            "frames": self.frames,
            "duration_s": duration,
        }
        if score is not None:
            out["score"] = score
        if staged_path is not None:
            out["staged_path"] = str(staged_path)
        return out


def normalize(text: str) -> str:
    return re.sub(r"\s+", " ", text.casefold()).strip()


def tokens(text: str) -> set[str]:
    ascii_tokens = set(re.findall(r"[a-z0-9]+", normalize(text)))
    cjk_tokens = set(re.findall(r"[\u4e00-\u9fff]+", text))
    return ascii_tokens | cjk_tokens


def read_frames(path: Path) -> int | None:
    try:
        with path.open("rb") as f:
            header = f.read(16)
            if len(header) != 16:
                return None
            magic, version, array_count = struct.unpack("<8sII", header)
            if magic != MAGIC or version != 1:
                return None
            for _ in range(array_count):
                raw = f.read(4)
                if len(raw) != 4:
                    return None
                (name_len,) = struct.unpack("<I", raw)
                name = f.read(name_len).decode("utf-8", errors="replace")
                raw = f.read(8)
                if len(raw) != 8:
                    return None
                _dtype, ndim = struct.unpack("<II", raw)
                shape = []
                if ndim:
                    raw = f.read(4 * ndim)
                    if len(raw) != 4 * ndim:
                        return None
                    shape = list(struct.unpack("<" + "I" * ndim, raw))
                raw = f.read(8)
                if len(raw) != 8:
                    return None
                (byte_count,) = struct.unpack("<Q", raw)
                if name == "joint_pos" and shape:
                    return int(shape[0])
                f.seek(byte_count, os.SEEK_CUR)
    except OSError:
        return None
    return None


def iter_presets(root: Path) -> Iterable[Preset]:
    for path in sorted(root.rglob("*.trk")):
        rel = path.relative_to(root)
        parts = rel.parts
        category = parts[-2] if len(parts) >= 2 else ""
        yield Preset(
            path=path.resolve(),
            relpath=str(rel),
            category=category,
            filename=path.name,
            frames=read_frames(path),
        )


def score_preset(query: str, preset: Preset) -> int:
    q_norm = normalize(query)
    q_tokens = tokens(query)
    category_norm = normalize(preset.category)
    filename_norm = normalize(Path(preset.filename).stem)
    aliases = CATEGORY_ALIASES.get(preset.category, [])

    score = 0
    if preset.category and preset.category in query:
        score += 100
    if category_norm and category_norm in q_norm:
        score += 80
    if filename_norm and filename_norm in q_norm:
        score += 20
    for alias in aliases:
        alias_norm = normalize(alias)
        if alias_norm and alias_norm in q_norm:
            score += 70 if " " in alias_norm else 45
        alias_tokens = tokens(alias)
        if alias_tokens and alias_tokens.issubset(q_tokens):
            score += 35
    category_tokens = tokens(preset.category)
    filename_tokens = tokens(Path(preset.filename).stem)
    score += 10 * len(q_tokens & category_tokens)
    score += 2 * len(q_tokens & filename_tokens)
    return score


def stage_preset(preset: Preset, stage_dir: Path) -> Path:
    stage_dir.mkdir(parents=True, exist_ok=True)
    safe_stem = re.sub(r"[^A-Za-z0-9_.-]+", "_", Path(preset.filename).stem).strip("._-")
    if not safe_stem:
        safe_stem = "motion"
    digest = hashlib.sha1(preset.relpath.encode("utf-8")).hexdigest()[:8]
    staged = stage_dir / f"et1-preset-{safe_stem}-{digest}.trk"
    shutil.copy2(preset.path, staged)
    return staged.resolve()


def list_presets(root: Path, fps: float, json_output: bool) -> int:
    presets = list(iter_presets(root))
    if json_output:
        print(json.dumps([p.to_dict(fps=fps) for p in presets], ensure_ascii=False))
        return 0

    current_category = None
    for preset in presets:
        if preset.category != current_category:
            current_category = preset.category
            print(f"\n{current_category}")
        meta = f"{preset.frames} frames" if preset.frames is not None else "unknown frames"
        print(f"  {preset.path} ({meta})")
    return 0


def search_presets(
    root: Path,
    query: str,
    limit: int,
    fps: float,
    json_output: bool,
    stage_dir: Path | None,
    path_only: bool,
) -> int:
    ranked = []
    for preset in iter_presets(root):
        score = score_preset(query, preset)
        if score > 0:
            ranked.append((score, preset))
    ranked.sort(key=lambda item: (-item[0], item[1].relpath))
    ranked = ranked[:limit]
    staged_path = stage_preset(ranked[0][1], stage_dir) if ranked and stage_dir is not None else None

    if path_only:
        if not ranked:
            return 0
        print(staged_path if staged_path is not None else ranked[0][1].path)
        return 0

    if json_output:
        out = []
        for idx, (score, preset) in enumerate(ranked):
            out.append(
                preset.to_dict(
                    score=score,
                    fps=fps,
                    staged_path=staged_path if idx == 0 else None,
                )
            )
        print(
            json.dumps(
                out,
                ensure_ascii=False,
            )
        )
        return 0

    if not ranked:
        print("No matching ET1 preset .trk found.")
        return 0
    for idx, (score, preset) in enumerate(ranked):
        duration = ""
        if preset.frames is not None and fps > 0:
            duration = f", {max(preset.frames - 1, 0) / fps:.2f}s @ {fps:g}fps"
        path = staged_path if idx == 0 and staged_path is not None else preset.path
        print(f"{score:3d}  {preset.category}  {path} ({preset.frames} frames{duration})")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Find ET1 preset .trk motions.")
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT, help="Preset .trk root directory.")
    parser.add_argument("--query", help="Natural-language action request to search.")
    parser.add_argument("--list", action="store_true", help="List all presets.")
    parser.add_argument("--limit", type=int, default=1, help="Maximum search results.")
    parser.add_argument("--fps", type=float, default=50.0, help="FPS for duration estimates.")
    parser.add_argument("--json", action="store_true", help="Emit JSON.")
    parser.add_argument("--path-only", action="store_true", help="Emit only the best preset path.")
    parser.add_argument("--stage", type=Path, help="Copy best match to this directory and return that path.")
    args = parser.parse_args()

    if not args.root.exists():
        raise SystemExit(f"Preset root does not exist: {args.root}")
    if args.list:
        return list_presets(args.root, args.fps, args.json)
    if args.query:
        return search_presets(
            args.root,
            args.query,
            max(args.limit, 1),
            args.fps,
            args.json,
            args.stage,
            args.path_only,
        )
    parser.error("provide --list or --query")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
