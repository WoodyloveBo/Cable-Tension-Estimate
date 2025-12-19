#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.ticker import MultipleLocator, FormatStrFormatter
import os

# ===================== 사용자 설정 =====================
CSV_FILE = "hover.csv"
OUTDIR = "fig_out_rmse_hover"

# ▶ 축 눈금 글자 크기
XTICK_FONTSIZE = 15
YTICK_FONTSIZE = 15

# ▶ 축 라벨 글자 크기
AXIS_LABEL_FONTSIZE = 14

# ▶ Figure 크기
FIGSIZE = (12, 4.5)

# ▶ y축 범위
Y_LIM = (0.0, 3.0)

os.makedirs(OUTDIR, exist_ok=True)

# ===================== Load CSV =====================
df = pd.read_csv(CSV_FILE)
payloads = df["payload"].values

# 표시할 metric들 (컬럼 suffix, 범례 라벨)
METRICS = [
    ("rmse_x",    "X"),
    ("rmse_y",    "Y"),
    ("rmse_z",    "Z"),
    ("rmse_norm", "Norm"),
]

# ▶ 색으로 component 구분
COLOR_BY_METRIC = {
    "rmse_x":    "red",
    "rmse_y":    "blue",
    "rmse_z":    "green",
    "rmse_norm": "gray",
}

# ===================== Plot =====================
fig, ax = plt.subplots(figsize=FIGSIZE)

num_payloads = len(payloads)
num_metrics = len(METRICS)

x = np.arange(num_payloads)

group_width = 0.8
bar_width = group_width / (num_metrics * 2.0)

for i, (metric_name, label_char) in enumerate(METRICS):
    cf1_vals = df[f"cf1_{metric_name}"].values
    cf2_vals = df[f"cf2_{metric_name}"].values
    color = COLOR_BY_METRIC[metric_name]

    metric_center_offset = (i - (num_metrics - 1) / 2.0) * (2.0 * bar_width)
    cf1_offset = metric_center_offset - bar_width / 2.0
    cf2_offset = metric_center_offset + bar_width / 2.0

    # CF1
    ax.bar(
        x + cf1_offset,
        cf1_vals,
        bar_width,
        color=color,
        edgecolor="black",
        linewidth=0.5,
        zorder=2,
    )

    # CF2
    ax.bar(
        x + cf2_offset,
        cf2_vals,
        bar_width,
        color=color,
        edgecolor="black",
        linewidth=0.5,
        hatch="////",
        zorder=2,
    )

# ===================== Axes 설정 =====================
ax.set_xticks(x)
ax.set_xticklabels(payloads)
ax.set_xlabel("Payload (g)", fontsize=AXIS_LABEL_FONTSIZE)

ax.set_ylim(*Y_LIM)

# ▶ y축 tick을 정수 간격(1.0)으로 고정
ax.yaxis.set_major_locator(MultipleLocator(1.0))
ax.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))

# ▶ tick label 크기
ax.tick_params(axis="x", labelsize=XTICK_FONTSIZE)
ax.tick_params(axis="y", labelsize=YTICK_FONTSIZE)

ax.grid(True, axis="y", alpha=0.3, zorder=0)

# ===================== Legend =====================
component_patches = [
    mpatches.Patch(
        facecolor=COLOR_BY_METRIC[m],
        edgecolor="black",
        label=lbl,
    )
    for m, lbl in METRICS
]

legend_components = ax.legend(
    handles=component_patches,
    loc="upper left",
    frameon=True,
    fontsize=15,
)

cf1_patch = mpatches.Patch(
    facecolor="lightgray",
    edgecolor="black",
    label="CF1",
)
cf2_patch = mpatches.Patch(
    facecolor="lightgray",
    edgecolor="black",
    hatch="////",
    label="CF2",
)

legend_drones = ax.legend(
    handles=[cf1_patch, cf2_patch],
    loc="upper right",
    frameon=True,
    fontsize=15,
)

ax.add_artist(legend_components)

# ===================== Save =====================
filename = "rmse_all_metrics_one_graph_color_components_hatch_cf"
for ext in [".png", ".pdf", ".svg"]:
    fig.savefig(
        os.path.join(OUTDIR, filename + ext),
        bbox_inches="tight",
    )

print(f"[✔] Saved combined figure: {filename}.[png|pdf|svg]")

plt.show()
