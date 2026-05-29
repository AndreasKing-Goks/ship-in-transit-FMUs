import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import truncnorm
from matplotlib.widgets import Slider

def truncated_normal_pdf(x, mean, sigma, lower, upper):
    a = (lower - mean) / sigma
    b = (upper - mean) / sigma
    return truncnorm.pdf(x, a, b, loc=mean, scale=sigma)

def truncated_normal_logpdf(x, mean, sigma, lower, upper):
    a = (lower - mean) / sigma
    b = (upper - mean) / sigma
    return truncnorm.logpdf(x, a, b, loc=mean, scale=sigma)

# Initial values in degrees
init_mean  = 0.0
init_sigma = 5.0
init_lower = -60.0
init_upper = 60.0

x = np.linspace(-180, 180, 1000)

fig, ax = plt.subplots(figsize=(9, 5))
plt.subplots_adjust(bottom=0.35)

pdf = truncated_normal_pdf(x, init_mean, init_sigma, init_lower, init_upper)
line, = ax.plot(x, pdf)

ax.axvline(init_lower, linestyle="--", label="lower")
ax.axvline(init_upper, linestyle="--", label="upper")
ax.axvline(init_mean, linestyle=":", label="mean")

ax.set_title("Truncated Normal PDF Tuning")
ax.set_xlabel("Angle (degrees)")
ax.set_ylabel("Probability density")
ax.legend()
ax.grid(True)

# Slider axes
ax_mean  = plt.axes([0.2, 0.24, 0.65, 0.03])
ax_sigma = plt.axes([0.2, 0.18, 0.65, 0.03])
ax_lower = plt.axes([0.2, 0.12, 0.65, 0.03])
ax_upper = plt.axes([0.2, 0.06, 0.65, 0.03])

slider_mean = Slider(ax_mean, "Mean", -180, 180, valinit=init_mean)
slider_sigma = Slider(ax_sigma, "Sigma", 1, 100, valinit=init_sigma)
slider_lower = Slider(ax_lower, "Lower", -180, 180, valinit=init_lower)
slider_upper = Slider(ax_upper, "Upper", -180, 180, valinit=init_upper)

def update(_):
    mean = slider_mean.val
    sigma = slider_sigma.val
    lower = slider_lower.val
    upper = slider_upper.val

    if lower >= upper:
        return

    pdf = truncated_normal_pdf(x, mean, sigma, lower, upper)
    logpdf = truncated_normal_logpdf(x, mean, sigma, lower, upper)

    log_likelihood = np.sum(logpdf[np.isfinite(logpdf)])

    line.set_ydata(pdf)

    ax.lines[1].set_xdata([lower, lower])
    ax.lines[2].set_xdata([upper, upper])
    ax.lines[3].set_xdata([mean, mean])

    ax.set_title(
        f"Truncated Normal PDF Tuning | Log Likelihood = {log_likelihood:.2f}"
    )

    ax.relim()
    ax.autoscale_view()
    fig.canvas.draw_idle()

slider_mean.on_changed(update)
slider_sigma.on_changed(update)
slider_lower.on_changed(update)
slider_upper.on_changed(update)

plt.show()

prev_scope_angle    = 0.0
scope_angle         = 0.0
max_reward_coeff    = 2
scope_angle_change  = scope_angle - prev_scope_angle
log_likelihood      = truncated_normal_logpdf(scope_angle_change, init_mean, init_sigma, init_lower, init_upper)
print(f"Log Likelihood of scope angle {scope_angle} given previous scope angle {prev_scope_angle}  : {log_likelihood}")

ll_sc0              = truncated_normal_logpdf(0, init_mean, init_sigma, init_lower, init_upper)
ll_sc_max           = truncated_normal_logpdf(60, init_mean, init_sigma, init_lower, init_upper)
ll_sc_max2          = truncated_normal_logpdf(-60, init_mean, init_sigma, init_lower, init_upper)
reward_component    = ((log_likelihood - ll_sc0) / (ll_sc0 - ll_sc_max)) * max_reward_coeff
print(f"Reward of scope angle {scope_angle} given previous scope angle {prev_scope_angle}          : {reward_component}")