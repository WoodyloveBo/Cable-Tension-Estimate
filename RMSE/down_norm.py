#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator, FormatStrFormatter
import os

# ===================== 사용자 설정 =====================
CSV_FILE = "down.csv"
OUTDIR = "fig_out_rmse_down_norm"
METRIC = "rmse_normalized"     # rmse_x / rmse_y / rmse_z / rmse_norm / rmse_normalized

# ▶ 폰트 크기
XTICK_FONTSIZE = 15
YTICK_FONTSIZE = 15
AXIS_LABEL_FONTSIZE = 13
LEGEND_FONTSIZE = 15

# ▶ y축 범위
Y_LIM = (0, 4)

# 폴더 생성
os.makedirs(OUTDIR, exist_ok=True)

# ===================== Load CSV =====================
df = pd.read_csv(CSV_FILE)

payloads = df["payload"].values
cf1_vals = df[f"cf1_{METRIC}"].values
cf2_vals = df[f"cf2_{METRIC}"].values

# ===================== Plot =====================
fig, ax = plt.subplots(figsize=(12, 4.5))

x = np.arange(len(payloads))
width = 0.35

# ===================== Bars =====================
# CF1 = 검정
ax.bar(
    x - width / 2,
    cf1_vals,
    width,
    label="CF1",
    color="black",
    edgecolor="black",
    linewidth=0.5,
    antialiased=False,
    zorder=2,
)

# CF2 = 회색
ax.bar(
    x + width / 2,
    cf2_vals,
    width,
    label="CF2",
    color="gray",
    edgecolor="gray",
    linewidth=0.5,
    antialiased=False,
    zorder=2,
)

# ===================== Axis 설정 =====================
ax.set_xticks(x)
ax.set_xticklabels(payloads)
ax.set_xlabel("Payload (g)", fontsize=AXIS_LABEL_FONTSIZE)

ax.set_ylim(*Y_LIM)

# ▶ y축 tick을 정수 간격(1.0)으로 고정
ax.yaxis.set_major_locator(MultipleLocator(1.0))
ax.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))  # 0, 1.0, 2.0 …

# ▶ tick label 크기
ax.tick_params(axis="x", labelsize=XTICK_FONTSIZE)
ax.tick_params(axis="y", labelsize=YTICK_FONTSIZE)

ax.grid(True, axis="y", alpha=0.3, zorder=0)

ax.legend(fontsize=LEGEND_FONTSIZE)

# ===================== Save =====================
filename = f"{METRIC}_CF1_CF2_no_shadow"
for ext in [".png", ".pdf", ".svg"]:
    fig.savefig(
        os.path.join(OUTDIR, filename + ext),
        bbox_inches="tight",
        dpi=300,
        facecolor="white",
        edgecolor="none",
    )

print(f"[✔] Saved: {filename}.[png|pdf|svg]")

plt.show()
