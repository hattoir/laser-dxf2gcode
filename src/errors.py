"""このツールの例外。CLI はこれらを捕まえて G-code を出さずに異常終了する。"""


class Dxf2GcodeError(Exception):
    """利用者に見せるべきエラーの基底クラス。"""


class DxfUnitError(Dxf2GcodeError):
    """DXF の単位が mm ではない / 不明。"""


class GeometryError(Dxf2GcodeError):
    """カーフ補正で輪郭が消える等、幾何的に加工できない。"""


class ConfigError(Dxf2GcodeError):
    """設定ファイルの不備。"""


class WorkAreaError(Dxf2GcodeError):
    """加工エリア外の座標が含まれている。安全上、絶対に G-code を出してはいけない。"""


class GcodeSafetyError(Dxf2GcodeError):
    """生成後の検証で安全規則違反(G0 中の発振、末尾 M5 欠落など)を検出した。"""
