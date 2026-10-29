"""将 YQY 导出的节点时程 BDF（兼容旧 CSV） 绘制为 PNG 和 PDF。"""

import argparse
import csv
import re
import os
import sys
from pathlib import Path


def read_history(data_path: Path):
    with data_path.open("r", encoding="utf-8-sig", newline="") as stream:
        if data_path.suffix.lower() == ".bdf":
            lines = [line.split() for line in stream if line.strip() and not line.lstrip().startswith("$")]
            if not lines:
                raise ValueError("BDF 文件没有有效数据")
            names = {"U1": "ux", "U2": "uy", "U3": "uz", "UR1": "rx", "UR2": "ry", "UR3": "rz"}
            columns = []
            node_id = ""
            for header in lines[0]:
                if header == "TIME":
                    columns.append("time")
                    continue
                match = re.fullmatch(r"N(\d+)-(U[123]|UR[123])", header)
                if not match or (node_id and node_id != match[1]):
                    raise ValueError("BDF 绘图需要同一节点的位移或转角数据")
                node_id = match[1]
                columns.append(names[match[2]])
            if any(len(row) != len(columns) for row in lines[1:]):
                raise ValueError("BDF 数据列数与表头不一致")
            rows = [dict(zip(columns, row), node_id=node_id) for row in lines[1:]]
        else:
            rows = list(csv.DictReader(stream))

    if not rows:
        raise ValueError("结果文件没有有效数据")

    def values(column):
        return [float(row[column]) for row in rows]

    time = values("time")
    node_id = rows[0].get("node_id", "")
    available = {key for key in rows[0] if key not in {"time", "node_id"}}
    preferred = ["ux", "uy", "uz", "rx", "ry", "rz", "twist", "twist_rate"]
    columns = [column for column in preferred if column in available]
    if not columns:
        columns = sorted(available)
    if not columns:
        raise ValueError("结果文件没有可绘制的数据列")
    return node_id, time, [(column, values(column)) for column in columns]


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
    plt.rcParams.update(
        {
            "axes.unicode_minus": False,
            "axes.edgecolor": "#B7C2D0",
            "axes.labelcolor": "#172033",
            "axes.linewidth": 0.8,
            "figure.facecolor": "#F5F7FA",
            "font.size": 10,
            "text.color": "#172033",
            "xtick.color": "#52606D",
            "ytick.color": "#52606D",
        }
    )


def create_plot(input_path: Path, output_path: Path):
    try:
        import matplotlib

        matplotlib.use("Agg")
        from matplotlib import font_manager, ticker
        import matplotlib.pyplot as plt
    except ImportError as error:
        raise RuntimeError("未安装 matplotlib，请执行：python -m pip install matplotlib") from error

    configure_chinese_font(plt, font_manager)
    node_id, time, series = read_history(input_path)
    source_name = input_path.stem
    if len(source_name) > 64:
        source_name = f"{source_name[:61]}..."
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure_height = max(3.6, 2.3 * len(series) + 1.4)
    figure, axes = plt.subplots(len(series), 1, figsize=(10.8, figure_height), sharex=True, squeeze=False)
    axes = axes[:, 0]
    colors = ["#2F80ED", "#00A896", "#D97706", "#7C3AED", "#DC2626", "#0891B2"]

    display_names = {
        "ux": "X 向位移 ux",
        "uy": "Y 向位移 uy",
        "uz": "Z 向位移 uz",
        "rx": "绕 X 转角 rx",
        "ry": "绕 Y 转角 ry",
        "rz": "绕 Z 转角 rz",
        "twist": "扭转角 twist",
        "twist_rate": "扭转角速度 twist_rate",
    }
    units = {"ux": "m", "uy": "m", "uz": "m", "rx": "rad", "ry": "rad", "rz": "rad", "twist": "rad", "twist_rate": "rad/s"}
    for index, (name, values) in enumerate(series):
        axis = axes[index]
        color = colors[index % len(colors)]
        axis.set_facecolor("#FFFFFF")
        axis.plot(time, values, color=color, linewidth=1.6, solid_capstyle="round")
        axis.axhline(0.0, color="#AEB8C5", linewidth=0.75, zorder=0)
        axis.set_ylabel(f"{display_names.get(name, name)}\n({units.get(name, '')})")
        axis.grid(True, color="#D9E0E8", linewidth=0.7, alpha=0.8)
        axis.margins(x=0.012, y=0.15)
        axis.tick_params(axis="both", which="major", labelsize=9, length=3.5, width=0.8)
        axis.yaxis.set_major_formatter(ticker.ScalarFormatter(useMathText=True))
        axis.ticklabel_format(axis="y", style="sci", scilimits=(-3, 3))
        axis.spines["top"].set_visible(False)
        axis.spines["right"].set_visible(False)

    axes[-1].set_xlabel("时间 t (s)")
    figure.suptitle(f"节点 {node_id} 时程响应", x=0.085, y=0.985, ha="left", fontsize=15, fontweight="bold")
    figure.text(0.085, 0.952, f"YQY 计算结果 · {source_name}", color="#687587", fontsize=9.5)
    figure.tight_layout(rect=(0.04, 0.02, 0.985, 0.92))
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
