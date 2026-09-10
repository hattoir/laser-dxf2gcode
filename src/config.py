"""machine.yaml と材料プロファイル(profiles/*.yaml)の読み込みと検証。"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import yaml

from .errors import ConfigError

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_MACHINE = ROOT / "machine.yaml"
PROFILES_DIR = ROOT / "profiles"

MAX_KERF = 1.0     # mm: これ以上のカーフは設定ミスとみなす
MAX_PASSES = 20


@dataclass
class Machine:
    name: str = "Creality Falcon2"
    laser_power_w: float = 22.0
    x_max: float = 400.0
    y_max: float = 415.0
    s_max: int = 1000
    max_rate_x: float = 25000.0
    max_rate_y: float = 8000.0
    accel_x: float = 500.0
    accel_y: float = 500.0
    junction_deviation: float = 0.01
    park_x: float = 0.0
    park_y: float = 0.0
    return_home: bool = True
    air_assist_gcode: bool = False
    unverified: list[str] = field(default_factory=list)

    def validate(self) -> None:
        if self.x_max <= 0 or self.y_max <= 0:
            raise ConfigError("machine: x_max / y_max は正の値にしてください")
        if int(self.s_max) != self.s_max or self.s_max <= 0:
            raise ConfigError("machine: s_max ($30) は正の整数にしてください")
        for k in ("max_rate_x", "max_rate_y", "accel_x", "accel_y", "junction_deviation"):
            if getattr(self, k) <= 0:
                raise ConfigError(f"machine: {k} は正の値にしてください")
        if not (0 <= self.park_x <= self.x_max and 0 <= self.park_y <= self.y_max):
            raise ConfigError("machine: park_x / park_y が加工エリア外です")


@dataclass
class PassSetting:
    power: float  # %
    feed: float   # mm/min


@dataclass
class LayerSettings:
    name: str
    power: float
    feed: float
    passes: int = 1
    through_cut: bool = True
    kerf_compensation: bool = True
    order: int = 10
    air_assist: bool = False
    per_pass: list[dict] = field(default_factory=list)

    def pass_settings(self) -> list[PassSetting]:
        """パスごとの (出力, 速度)。per_pass の i 番目で i パス目を上書きする。"""
        out = []
        for i in range(self.passes):
            o = self.per_pass[i] if i < len(self.per_pass) and self.per_pass[i] else {}
            out.append(PassSetting(float(o.get("power", self.power)), float(o.get("feed", self.feed))))
        return out

    def validate(self, where: str) -> None:
        if int(self.passes) != self.passes or not (1 <= self.passes <= MAX_PASSES):
            raise ConfigError(f"{where}: passes は 1〜{MAX_PASSES} の整数にしてください")
        if len(self.per_pass) > self.passes:
            raise ConfigError(f"{where}: per_pass の要素数が passes より多いです")
        for i, p in enumerate(self.pass_settings(), 1):
            if p.feed <= 0:
                raise ConfigError(f"{where}: {i} パス目の feed が 0 以下です")
            # 出力 % の範囲外は G-code 生成時に S 値をクランプして警告する(仕様)


@dataclass
class Profile:
    name: str
    kerf: float
    default_layer: str | None
    layers: dict[str, LayerSettings]
    path: Path | None = None

    def settings_for(self, layer_name: str) -> tuple[LayerSettings, bool]:
        """DXF のレイヤー名に対応する設定。一致しなければ default_layer (matched=False)。"""
        for k, v in self.layers.items():
            if k.lower() == layer_name.lower():
                return v, True
        if self.default_layer is None:
            raise ConfigError(
                f"DXF のレイヤー '{layer_name}' に対応する設定がプロファイル '{self.name}' にありません"
                "(default_layer も未設定)")
        return self.layers[self.default_layer], False

    def validate(self) -> None:
        if not (0.0 <= self.kerf <= MAX_KERF):
            raise ConfigError(f"profile {self.name}: kerf {self.kerf} が 0〜{MAX_KERF}mm の範囲外です")
        if not self.layers:
            raise ConfigError(f"profile {self.name}: layers が空です")
        if self.default_layer is not None and self.default_layer not in self.layers:
            raise ConfigError(f"profile {self.name}: default_layer '{self.default_layer}' が layers にありません")
        for k, v in self.layers.items():
            v.validate(f"profile {self.name} / layer {k}")


def _load_yaml(path: Path) -> dict:
    try:
        with open(path, encoding="utf-8") as f:
            data = yaml.safe_load(f)
    except FileNotFoundError as exc:
        raise ConfigError(f"設定ファイルがありません: {path}") from exc
    except yaml.YAMLError as exc:
        raise ConfigError(f"YAML の書式エラー: {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise ConfigError(f"設定ファイルの中身が空か不正です: {path}")
    return data


def load_machine(path: str | Path | None = None) -> Machine:
    path = Path(path) if path else DEFAULT_MACHINE
    data = _load_yaml(path)
    m = data.get("machine", {})
    known = set(Machine.__dataclass_fields__) - {"unverified"}
    unknown = set(m) - known
    if unknown:
        raise ConfigError(f"machine.yaml に不明なキー: {sorted(unknown)}")
    machine = Machine(**m, unverified=list(data.get("unverified") or []))
    machine.validate()
    return machine


def resolve_profile_path(name_or_path: str) -> Path:
    p = Path(name_or_path)
    if p.suffix in (".yaml", ".yml"):
        return p
    return PROFILES_DIR / f"{name_or_path}.yaml"


def load_profile(name_or_path: str) -> Profile:
    path = resolve_profile_path(name_or_path)
    data = _load_yaml(path)
    name = Path(name_or_path).stem
    if name in data:
        body = data[name]
    elif len(data) == 1:
        name, body = next(iter(data.items()))
    else:
        raise ConfigError(f"{path} に プロファイル '{name}' がありません")
    return profile_from_dict(name, body, path)


def profile_from_dict(name: str, body: dict, path: Path | None = None) -> Profile:
    layers = {}
    for lname, ld in (body.get("layers") or {}).items():
        try:
            layers[lname] = LayerSettings(name=lname, **ld)
        except TypeError as exc:
            raise ConfigError(f"profile {name} / layer {lname}: {exc}") from exc
    prof = Profile(name=name, kerf=float(body.get("kerf", 0.0)),
                   default_layer=body.get("default_layer"), layers=layers, path=path)
    prof.validate()
    return prof
