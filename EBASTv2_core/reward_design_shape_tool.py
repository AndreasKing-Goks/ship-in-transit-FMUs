from pathlib import Path
import sys
import os

import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import truncnorm

from matplotlib.ticker import FuncFormatter

## PATH HELPER (OBLIGATORY)
# project root = two levels up from this file
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from EBASTv2_core.reward_designs import (
    RewardDesign1, RewardDesign2, RewardDesign3,
    RewardDesign4, RewardDesign5, RewardDesign6,
    RewardDesign7
)

# === Customizable boundaries for X axis ===
x_min = 0
x_max = 10000

# x_min = -60
# x_max = 60

# x_min = 0
# x_max = 2000

# Generate x values
x = np.linspace(x_min, x_max, 1000)

# Instantiate reward functions using parameters that mimic the paper's figure
designs = [
    RewardDesign1(target=0, offset_param=500),
    RewardDesign2(target=1000, offset_param1=10000, offset_param2=10000000),
    RewardDesign3(target=1000, offset_param=150000),
    RewardDesign4(target=100, offset_param=15000000),
    RewardDesign5(target_bound_low=40, target_bound_high=60, offset_param=100),
    RewardDesign6(target1=30, target2=70, second_peak=0.8, flat_zone=0.5,
                  offset_param1=100, offset_param2=50, offset_param3=50, offset_param4=100),
    RewardDesign7(collision_zone_radius=100.0)
]

test_1 = False
# test_1 = True

if test_1:
    # Plotting figure to match the paper style (Fig. 2)
    fig, axs = plt.subplots(2, 3, figsize=(14, 6))
    axs = axs.flatten()
    titles = [f"(a) Design {i+1}" for i in range(6)]

    for i, design in enumerate(designs):
        y = [design(float(xi)) for xi in x]  # Scalar calls
        axs[i].plot(x, y, linewidth=2)
        axs[i].set_title(titles[i], fontsize=12)

        axs[i].set_xlim(x_min, x_max)
        axs[i].set_ylim(-.1, 1.1)

        # Dynamic ticks
        axs[i].set_xticks(np.linspace(x_min, x_max, 6))  # 6 evenly spaced ticks
        axs[i].set_yticks([.0, 0.5, 1.0])

        axs[i].set_xlabel("Input value")
        axs[i].set_ylabel("Reward")

        axs[i].grid(True, which='both', linestyle='--', linewidth=0.5, alpha=0.7)

    fig.suptitle("Evaluation Functions – Corresponding to Goto et al. (2023)", fontsize=14)
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    plt.show()

test_2 = False
# test_2 = True

if test_2:
    design = designs[0]

    y = [design(float(xi)) for xi in x]
    plt.figure(figsize=(7, 6.8))
    plt.plot(x, y)
    plt.xlim(x_min, x_max)
    plt.ylim(-.1, 1.1)

    plt.xticks(np.linspace(x_min, x_max, 6))
    plt.yticks([.0, 0.5, 1.0])

    plt.xlabel('Input value')
    plt.ylabel('Reward')
    plt.grid()
    # plt.tight_layout()
    plt.show()

nd_ = False
nd_ = True

if nd_:
    design = designs[6]
    
    y = [design(float(xi)) for xi in x]
    fig, ax = plt.subplots(figsize=(7, 6.8))
    ax.plot(x, y, linewidth=2.5)
    ax.axvline(x=100,linestyle='--',linewidth=2,label=r'$D_{\mathrm{collision}}$')
    ax.set_xlim(x_min, x_max)
    ax.xaxis.set_major_formatter(FuncFormatter(lambda value, pos: f'{value / 1000:g}'))
    ax.set_xlabel(r'OS/TS distance (km)', fontsize=13, labelpad=8)
    ax.set_ylabel('Reward', fontsize=13, labelpad=8)
    ax.annotate(r'$D_{\mathrm{collision}}=0.1\,\mathrm{km}$',xy=(100, -5),xytext=(900, -4.5),fontsize=13,arrowprops=dict(arrowstyle='->'))
    ax.tick_params(axis='both', labelsize=14)
    ax.grid(True,linestyle='--',linewidth=0.7,alpha=0.5)
    ax.legend(fontsize=11, frameon=False, loc='upper right')
    fig.subplots_adjust(left=0.16,right=0.97,bottom=0.14,top=0.97)
    plt.show()
    
ic_ = False
# ic_ = True

if ic_:
    design = designs[0]
    
    y = [(design(float(xi))-1) for xi in x]
    fig, ax = plt.subplots(figsize=(7, 6.8))
    ax.plot(x, y, linewidth=2.5)
    ax.axvline(x=0,linestyle='--',linewidth=2,label=r'$\mathrm{mean}\,\Delta\psi^{\text{sc}}_{\text{ic}}$')
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(-1.1, 0.1)
    ax.set_xlabel(r'Intercepting Scope Angle Error $\Delta\psi^{\text{sc}}_{\text{ic}}$ ($\text{deg}$)', fontsize=11, labelpad=8)
    ax.set_ylabel('Reward', fontsize=13, labelpad=8)
    ax.tick_params(axis='both', labelsize=14)
    ax.grid(True,linestyle='--',linewidth=0.7,alpha=0.5)
    ax.legend(fontsize=11, frameon=False, loc='upper right')
    fig.subplots_adjust(left=0.19,right=0.97,bottom=0.14,top=0.97)
    plt.show()
    
lpdf_plot_norm = False
# lpdf_plot_norm = True

def logprior_scope_angle_change(
    scope_angle_change,
    mean_change,
    sigma=5.0,
    theta=60.0
):
    """
        Truncated normal distribution prior for the sampled scope angle change 
        around the mean 0. The idea is that RL agent would ideally keep  it 
        course heading the same as the previous scope angle it used. The larger
        the change of sampled scope angle to the previously sampled scope angle,
        the more unlikely it is
        
        (a, b) is a measure of how many standard deviations away the lower and
        upper bounds from the mean value.

        Support:
            [-theta, theta] degrees
    """
    
    # Upper and Lower bound
    lower = -theta
    upper = theta

    # Normalized
    a = (lower - mean_change) / sigma
    b = (upper - mean_change) / sigma

    return truncnorm.logpdf(
        scope_angle_change,
        a,
        b,
        loc=mean_change,
        scale=sigma
    )

if lpdf_plot_norm:
    init_mean  = 0.0
    init_sigma = 5.0
    theta      = 60

    low_log_pdf  = logprior_scope_angle_change(0.0, init_mean, init_sigma, theta)
    high_log_pdf = logprior_scope_angle_change(60.0, init_mean, init_sigma, theta)

    design = lambda x: (
        logprior_scope_angle_change(x, init_mean, init_sigma, theta) - low_log_pdf
    ) / (low_log_pdf - high_log_pdf)
    
    y = [design(float(xi)) for xi in x]
    fig, ax = plt.subplots(figsize=(7, 6.8))
    ax.plot(x, y, linewidth=2.5)
    ax.axvline(x=0,linestyle='--',linewidth=2,label=r'$\mathrm{mean}\,\Delta\psi^{\text{sc}}$')
    ax.set_xlim(x_min, x_max)
    ax.set_xlabel(r'Scope angle changes $\Delta\psi^{\text{sc}}$ ($\text{deg}$)', fontsize=13, labelpad=8)
    ax.set_ylabel('Reward', fontsize=13, labelpad=8)
    ax.tick_params(axis='both', labelsize=14)
    ax.grid(True,linestyle='--',linewidth=0.7,alpha=0.5)
    ax.legend(fontsize=11, frameon=False, loc='upper right')
    fig.subplots_adjust(left=0.19,right=0.97,bottom=0.14,top=0.97)
    plt.show()