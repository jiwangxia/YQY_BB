"""将 YQY 导出的节点时程 CSV 绘制为 PNG 和 PDF。"""

import argparse
import csv
import os
import sys
from pathlib import Path


def read_history(csv_path: Path):
    with csv_path.open("r", encoding="utf-8-sig", newline="") as stream:
        rows = list(csv.DictReader(stream))

    if not rows:
        raise ValueError("CSV 文件没有有效数据")

    def values(column):
        return [float(row[column]) for row in rows]

    time = values("time")
    available = {key for key in rows[0] if key not in {"time", "node_id"}}
    preferred = ["ux", "uy", "uz", "twist", "twist_rate"]
    columns = [column for column in preferred if column in available]
    if not columns:
        columns = sorted(available)
    if not columns:
        raise ValueError("CSV 文件没有可绘制的数据列")
    return time, [(column, values(column)) for column in columns]


def configure_chinese_font(plt, font_manager):
    """优先加载 Windows 中文字体，避免 Matplotlib 用默认字体显示方框。"""
    windows_fonts = Path(os.environ.get("WINDIR", "C:/Windows")) / "Fonts"
    font_files = [
        windows_fonts / "msyh.ttc",
        windows_fonts / "msyhbd.ttc",
        windows_fonts / "simhei.ttf",
        windows_fonts / "simsun.ttc",
        windows_fonts / "Deng.ttf",
    ]
    for font_file in font_files:
        if not font_file.is_file():
            continue
        try:
            font_manager.fontManager.addfont(str(font_file))
            font_name = font_manager.FontProperties(fname=str(font_file)).get_name()
            plt.rcParams["font.family"] = font_name
            break
        except (OSError, RuntimeError):
            continue
    plt.rcParams["axes.unicode_minus"] = False


def create_plot(input_path: Path, output_path: Path):
    try:
        import matplotlib

        matplotlib.use("Agg")
        from matplotlib import font_manager
        import matplotlib.pyplot as plt
    except ImportError as error:
        raise RuntimeError("未安装 matplotlib，请执行：python -m pip install matplotlib") from error

    configure_chinese_font(plt, font_manager)
    time, series = read_history(input_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure, axes = plt.subplots(len(series), 1, figsize=(10, 7), sharex=True, squeeze=False)
    axes = axes[:, 0]
    colors = ["#2F80ED", "#00A896", "#D97706", "#7C3AED", "#DC2626"]

    display_names = {
        "ux": "X 向位移 ux",
        "uy": "Y 向位移 uy",
        "uz": "Z 向位移 uz",
        "twist": "扭转角 twist",
        "twist_rate": "扭转角速度 twist_rate",
    }
    for index, (name, values) in enumerate(series):
        axis = axes[index]
        axis.plot(time, values, color=colors[index % len(colors)], linewidth=1.25)
        axis.set_ylabel(display_names.get(name, name))
        axis.grid(True, color="#D7DEE8", linewidth=0.7, alpha=0.85)
        axis.spines["top"].set_visible(False)
        axis.spines["right"].set_visible(False)

    axes[-1].set_xlabel("时间 (s)")
    figure.suptitle(f"YQY 结果时程 · {input_path.stem}", fontsize=13, fontweight="bold")
    figure.tight_layout(rect=(0, 0, 1, 0.96))
    figure.savefig(output_path, dpi=180, bbox_inches="tight")
    figure.savefig(output_path.with_suffix(".pdf"), bbox_inches="tight")
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description="绘制 YQY 节点时程结果")
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    arguments = parser.parse_args()

    if not arguments.input.is_file():
        print(f"输入文件不存在：{arguments.input}", file=sys.stderr)
        return 2
    try:
        create_plot(arguments.input, arguments.output)
    except (OSError, ValueError, RuntimeError, KeyError, TypeError) as error:
        print(f"绘图失败：{error}", file=sys.stderr)
        return 1
    print(f"PNG：{arguments.output}")
    print(f"PDF：{arguments.output.with_suffix('.pdf')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
