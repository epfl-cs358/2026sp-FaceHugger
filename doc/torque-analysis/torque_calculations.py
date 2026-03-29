# ---
# jupyter:
#   jupytext:
#     formats: ipynb,py:percent
#     text_representation:
#       extension: .py
#       format_name: percent
#       format_version: '1.3'
#       jupytext_version: 1.19.1
#   kernelspec:
#     display_name: .venv
#     language: python
#     name: python3
# ---

# %% [markdown]
# # Quadruped Leg — Torque Calculations
#
# This notebook determines the **maximum body weight** a quadruped leg can
# support, given its servo torque ratings and link geometry. It computes
# static pitch torques (hip and knee) and dynamic yaw torque (shoulder),
# then inverts the equations to find the per-leg load capacity.
#
# The leg is modelled as a **3-link serial chain**:
# - $L_1$ - shoulder to hip joint ($s_{L2}$ pivot)
# - $L_2$ - hip joint to knee joint ($s_{L3}$ pivot)
# - $L_3$ - knee joint to tip (foot contact point)
#
# <div style="display:flex;gap:32px;justify-content:center;align-items:flex-start;margin:16px 0">
#   <figure style="text-align:center;margin:0">
#     <img src="img/robot-leg-setup-sketch-.png" width="340" alt="Leg sketch">
#     <figcaption style="font-size:12px;color:#888;margin-top:6px">3-link leg model (L<sub>1</sub>, L<sub>2</sub>, L<sub>3</sub>)</figcaption>
#   </figure>
#   <figure style="text-align:center;margin:0">
#     <img src="img/wikipedia-plane-pitch-yaw-roll.png" width="300" alt="Pitch Yaw Roll axes">
#     <figcaption style="font-size:12px;color:#888;margin-top:6px">Rotation axes — pitch, yaw, roll</figcaption>
#   </figure>
# </div>
#
# Analysis approach: work **tip to base**. Each inner motor must support
# all masses beyond it at their respective distances from that pivot.

# %% [markdown]
# ## Rotation axes — vocabulary
#
# | Axis  | Rotation          | This leg        |
# |-------|-------------------|--------------------|
# | Pitch | nose up / down    | hip and knee    |
# | Yaw   | nose left / right | shoulder        |
# | Roll  | wingtip up / down | not used        |
#
# **Memory aid:** *Pitch a ball* (arm swings up/down). *Yawn* left and right.
# *Roll* a barrel along its long axis.
#
# Hip ($s_{L2}$) and knee ($s_{L3}$) are **pitch** axes - gravity acts perpendicular to
# them, so they must actively resist it. The shoulder ($s_{L1}$) is a **yaw** axis -
# vertical, so gravity is parallel to it and creates zero static torque.
# The shoulder motor instead resists **rotational inertia** during acceleration.

# %%
import math
from pathlib import Path

import ipywidgets as widgets
import matplotlib.pyplot as plt
import yaml
from IPython.display import display

# --- Display config ---
# Adjust this to match your screen / notebook zoom level.
FIGURE_SIZE_IN = 5  # leg diagram: width and height in inches (square)

# %%
# --- Config loader ---
try:
    _CONFIG_PATH = Path(__file__).parent / "config.yaml"
except NameError:
    # __file__ is not defined in a notebook kernel; cwd is this directory
    _CONFIG_PATH = Path.cwd() / "config.yaml"
_config_cache = None


def load_config() -> dict:
    global _config_cache
    if _config_cache is None:
        with open(_CONFIG_PATH) as f:
            _config_cache = yaml.safe_load(f)
    return _config_cache


def get_config(key: str, default=None):
    """Dot-notation accessor. e.g. get_config('links.L2.length_cm', 8.0)"""
    cfg = load_config()
    keys = key.split(".")
    val = cfg
    try:
        for k in keys:
            val = val[k]
        return val
    except (KeyError, TypeError):
        return default


def link_mass_kg(link_name: str) -> float:
    """Estimate link mass from config geometry and density, with margin."""
    l = get_config(f"links.{link_name}.length_cm")
    w = get_config(f"links.{link_name}.width_cm")
    h = get_config(f"links.{link_name}.height_cm")
    rho = get_config(f"links.{link_name}.density_g_cm3", 1.24)
    margin = get_config(f"links.{link_name}.mass_margin", 0.15)
    volume_cm3 = l * w * h
    mass_g = volume_cm3 * rho * (1 + margin)
    return mass_g / 1000.0  # kg


# Sanity-check mass estimates
for name in ["L1", "L2", "L3"]:
    print(
        f"  {name}: {link_mass_kg(name) * 1000:.1f} g  "
        f"(length={get_config(f'links.{name}.length_cm')} cm)"
    )
print(f"  Servo mass: {get_config('servos.s_L2.mass_kg') * 1000:.0f} g")

# %% [markdown]
# ## Pitch axis — static torque
#
# For a mass $m$ at horizontal distance $d$ from a pitch-axis pivot,
# the gravitational torque the motor must resist is:
#
# $$\tau = m \cdot g \cdot d \cdot \cos(\theta)$$
#
# where $\theta$ is the angle of the arm from horizontal.
#
# **Worst case:** $\theta = 0$ — leg fully horizontal, $\cos(0) = 1$.
#
# For a **distributed link mass**, the force acts at the **centre of mass**
# (midpoint for a uniform beam), so the effective distance is $L/2$.
#
# Units: lengths in **cm**, masses in **kg**, torques in **kg·cm**
# (1 kg·cm = 0.098 N·m).

# %%
# --- s_L3: Knee motor (pitch) ---
# Pivot at end of L2. Sees: L3 link mass + tip load.


def torque_knee(
    r3_cm: float, theta_deg: float, m_L3_kg: float, m_tip_kg: float = 0.0
) -> float:
    """
    Static torque at the knee motor (s_L3) in kg·cm.

    Parameters
    ----------
    r3_cm     : length of L3 (cm)
    theta_deg : angle of leg from horizontal (degrees)
    m_L3_kg   : mass of link L3 (kg)
    m_tip_kg  : mass at the tip (kg)
    """
    cos_t = math.cos(math.radians(theta_deg))
    return (m_L3_kg * (r3_cm / 2.0) + m_tip_kg * r3_cm) * cos_t


r3 = get_config("links.L3.length_cm", 6.0)
m_L3 = link_mass_kg("L3")
m_tip = get_config("tip.mass_kg", 0.0)
rated = get_config("servos.s_L3.torque_rated_kg_cm", 15.0)

tau_knee_wc = torque_knee(r3, theta_deg=0.0, m_L3_kg=m_L3, m_tip_kg=m_tip)
print(f"s_L3 (knee) — worst case torque : {tau_knee_wc:.2f} kg·cm")
print(f"s_L3 rated torque               : {rated:.1f} kg·cm")
print(f"Safety margin                   : {rated / tau_knee_wc:.1f}x")

# %% [markdown]
# ### $s_{L3}$ - knee motor
#
# The knee motor only supports **link $L_3$ and the tip**. At these link lengths
# and PLA masses, the required torque is a small fraction of the servo rating.
# The distributed mass of $L_3$ acts at its midpoint ($L_3/2$).

# %%
# --- s_L2: Hip motor (pitch) ---
# Pivot at end of L1. Sees: L2 link, knee servo, L3 link, tip.


def torque_hip(
    r2_cm: float,
    r3_cm: float,
    theta_hip_deg: float,
    theta_knee_deg: float,
    m_L2_kg: float,
    m_L3_kg: float,
    m_servo_knee_kg: float,
    m_tip_kg: float = 0.0,
) -> float:
    """
    Static torque at the hip motor (s_L2) in kg·cm.

    Uses independent hip and knee angles to correctly model the articulated
    leg geometry.  L2-attached masses are projected by cos(hip); L3-attached
    masses are projected by cos(knee).

    Parameters
    ----------
    r2_cm           : length of L2 (cm)
    r3_cm           : length of L3 (cm)
    theta_hip_deg   : hip angle from horizontal (degrees)
    theta_knee_deg  : knee angle from horizontal (degrees)
    m_L2_kg         : mass of link L2 (kg)
    m_L3_kg         : mass of link L3 (kg)
    m_servo_knee_kg : mass of the knee servo body (kg)
    m_tip_kg        : mass at the tip (kg)
    """
    cos_h = math.cos(math.radians(theta_hip_deg))
    cos_k = math.cos(math.radians(theta_knee_deg))
    tau = (
        m_L2_kg * (r2_cm / 2.0) * cos_h
        + m_servo_knee_kg * r2_cm * cos_h
        + m_L3_kg * (r2_cm * cos_h + r3_cm / 2.0 * cos_k)
        + m_tip_kg * (r2_cm * cos_h + r3_cm * cos_k)
    )
    return tau


r2 = get_config("links.L2.length_cm", 8.0)
m_L2 = link_mass_kg("L2")
m_servo_knee = get_config("servos.s_L3.mass_kg", 0.072)
rated_hip = get_config("servos.s_L2.torque_rated_kg_cm", 15.0)

tau_hip_wc = torque_hip(
    r2,
    r3,
    theta_hip_deg=0.0,
    theta_knee_deg=0.0,
    m_L2_kg=m_L2,
    m_L3_kg=m_L3,
    m_servo_knee_kg=m_servo_knee,
    m_tip_kg=m_tip,
)
print(f"s_L2 (hip) — worst case torque  : {tau_hip_wc:.2f} kg·cm")
print(f"s_L2 rated torque               : {rated_hip:.1f} kg·cm")
print(f"Safety margin                   : {rated_hip / tau_hip_wc:.1f}x")

# %% [markdown]
# ### $s_{L2}$ - hip motor
#
# The hip motor carries the **full distal chain**: link $L_2$, the knee servo body,
# link $L_3$, and the tip. Each mass is weighted by its distance from the hip pivot.
#
# Note the knee servo mass (72 g) contributes significantly - it sits at the
# full length of $L_2$ from the hip pivot, giving it maximum leverage.
# This is why keeping servos close to the base is important in legged robots.

# %% [markdown]
# ## Yaw axis — dynamic torque
#
# The shoulder ($s_{L1}$) rotates about a **vertical axis**. Because gravity acts
# downward (parallel to the axis), it creates **zero static torque** — the motor
# does not need to resist gravity regardless of leg position.
#
# What the shoulder motor *does* resist is **rotational inertia** when the leg
# accelerates angularly. Newton's second law for rotation gives:
#
# $$\tau = I \cdot \alpha$$
#
# where $\alpha = \Delta\omega / \Delta t$ is the angular acceleration (rad/s²).
#
# The **moment of inertia** of the leg about the shoulder yaw axis is:
#
# $$I = \sum_i m_i r_i^2$$
#
# with each component modelled as a point mass at its distance $r_i$ from the
# shoulder pivot. Units: kg·cm² (convert to kg·m² before applying $\tau = I\alpha$).

# %%
# --- s_L1: Shoulder motor (yaw) ---
# Vertical axis. No static gravity torque. Resists rotational inertia only.


def moment_of_inertia_leg(
    r1_cm,
    r2_cm,
    r3_cm,
    m_L1_kg,
    m_L2_kg,
    m_L3_kg,
    m_servo_hip_kg,
    m_servo_knee_kg,
    m_tip_kg=0.0,
):
    """
    Moment of inertia of the full leg about the shoulder yaw axis (kg·cm²).

    Links are modelled as uniform rods using the parallel axis theorem:
        I_link = (1/12) * m * L² + m * d²
    where L is the link length and d is the distance from the yaw axis to
    the link's centre of mass.  Servos and tip remain point masses (I = m·d²).
    """
    # Links: parallel axis theorem  (I_cm + m * d²)
    d_L1 = r1_cm / 2.0
    d_L2 = r1_cm + r2_cm / 2.0
    d_L3 = r1_cm + r2_cm + r3_cm / 2.0

    I_links = (
        (1.0 / 12.0) * m_L1_kg * r1_cm**2
        + m_L1_kg * d_L1**2
        + (1.0 / 12.0) * m_L2_kg * r2_cm**2
        + m_L2_kg * d_L2**2
        + (1.0 / 12.0) * m_L3_kg * r3_cm**2
        + m_L3_kg * d_L3**2
    )

    # Point masses: servos and tip
    I_points = (
        m_servo_hip_kg * r1_cm**2
        + m_servo_knee_kg * (r1_cm + r2_cm) ** 2
        + m_tip_kg * (r1_cm + r2_cm + r3_cm) ** 2
    )

    return I_links + I_points


def torque_yaw(I_kg_cm2: float, target_deg_per_s: float, ramp_time_s: float) -> float:
    """
    Dynamic torque to accelerate leg to target angular velocity
    within ramp_time_s. Returns kg·cm.
    """
    alpha = math.radians(target_deg_per_s) / ramp_time_s  # rad/s²
    I_kg_m2 = I_kg_cm2 * 1e-4
    tau_Nm = I_kg_m2 * alpha
    return tau_Nm * 10.197  # convert N·m to kg·cm


r1 = get_config("links.L1.length_cm", 8.0)
m_L1 = link_mass_kg("L1")
m_servo_hip = get_config("servos.s_L2.mass_kg", 0.072)
rated_yaw = get_config("servos.s_L1.torque_rated_kg_cm", 15.0)

I_total = moment_of_inertia_leg(
    r1, r2, r3, m_L1, m_L2, m_L3, m_servo_hip, m_servo_knee, m_tip
)

target_speed = 60.0  # deg/s — reasonable leg sweep speed
ramp_time = 0.2  # s — time to reach that speed from rest

tau_yaw_val = torque_yaw(I_total, target_speed, ramp_time)

print(f"Total leg inertia (yaw axis)    : {I_total:.2f} kg·cm²")
print(f"Target speed / ramp time        : {target_speed} deg/s in {ramp_time} s")
print(f"s_L1 (shoulder) dynamic torque  : {tau_yaw_val:.3f} kg·cm")
print(f"s_L1 rated torque               : {rated_yaw:.1f} kg·cm")
print(f"Safety margin                   : {rated_yaw / tau_yaw_val:.1f}x")

# %% [markdown]
# ## Interactive explorer
#
# The widget below answers: **given this leg geometry, what is the maximum
# robot body weight these servos can support?**
#
# **How it works:** the torque equations are inverted — instead of computing
# torque from a known load, we ask what tip load would saturate each motor.
# The binding motor (whichever saturates first) sets the limit.
# Body weight = tip load × number of legs in stance.
#
# **Sliders:**
# - *$L_2$ / $L_3$ length* - link lengths in cm
# - *Hip / Knee angle* — degrees from horizontal; 0° is worst case (fully extended),
#   higher angles mean the leg is bent and motors have an easier time
# - *Yaw ramp* — how fast the shoulder accelerates during a swing; only affects
#   the shoulder row, not the body weight calculation
#
# **Capacity bars** — how hard each motor is working at the maximum load.
# The limiting motor always shows 100%. A motor at 40% has headroom to spare.
#
# **Servo parameters** — editable rated torque and mass; update these when
# you confirm the actual servo model.

# %% [markdown]
# ### $s_{L1}$ - shoulder yaw motor
#
# Because the leg is lightweight **PETG** at short radii, rotational inertia is low
# and the dynamic torque demand is a small fraction of the rating.
# The yaw motor is **not the limiting factor** for this leg design.
#
# If this ever becomes a concern, the levers are:
# - Reduce distal mass (especially servos far from shoulder)
# - Increase ramp time (slower acceleration profile)

# %%
# --- Interactive widget ---
# Top-left: sliders (L2, L3 lengths; theta; yaw ramp time)
# Top-right: torque table with rated comparison for all three motors
# Bottom: 2D leg diagram at current angle


def build_widget():
    slider_style = {"description_width": "120px"}
    slider_layout = widgets.Layout(width="340px")

    sl_r2 = widgets.FloatSlider(
        value=get_config("links.L2.length_cm", 8.0),
        min=3.0,
        max=10.0,
        step=0.5,
        description="L2 length (cm)",
        style=slider_style,
        layout=slider_layout,
    )
    sl_r3 = widgets.FloatSlider(
        value=get_config("links.L3.length_cm", 6.0),
        min=3.0,
        max=10.0,
        step=0.5,
        description="L3 length (cm)",
        style=slider_style,
        layout=slider_layout,
    )
    sl_theta_hip = widgets.FloatSlider(
        value=0.0,
        min=0.0,
        max=135.0,
        step=5.0,
        description="Hip angle (deg)",
        style=slider_style,
        layout=slider_layout,
    )
    sl_theta_kne = widgets.FloatSlider(
        value=0.0,
        min=0.0,
        max=135.0,
        step=5.0,
        description="Knee angle (deg)",
        style=slider_style,
        layout=slider_layout,
    )
    sl_ramp = widgets.FloatSlider(
        value=0.2,
        min=0.05,
        max=1.0,
        step=0.05,
        description="Yaw ramp (s)",
        style=slider_style,
        layout=slider_layout,
    )

    # Servo parameter number boxes (τ rated in kg·cm, mass in g)
    _trq_col = widgets.Layout(width="95px")
    _m_col = widgets.Layout(width="75px")

    def _trq(servo_key):
        return widgets.BoundedFloatText(
            value=get_config(f"servos.{servo_key}.torque_rated_kg_cm", 15.0),
            min=0.1,
            max=200.0,
            step=0.5,
            layout=_trq_col,
        )

    def _mass(servo_key):
        return widgets.BoundedFloatText(
            value=round(get_config(f"servos.{servo_key}.mass_kg", 0.072) * 1000),
            min=1,
            max=500,
            step=1,
            layout=_m_col,
        )

    nt_trq_h, nt_m_h = _trq("s_L2"), _mass("s_L2")  # hip
    nt_trq_k, nt_m_k = _trq("s_L3"), _mass("s_L3")  # knee
    nt_trq_sh, nt_m_sh = _trq("s_L1"), _mass("s_L1")  # shoulder

    _lbl_layout = widgets.Layout(width="90px")
    servo_header = widgets.HTML(
        "<div style='display:flex;gap:4px;font-size:11px;color:#888;font-weight:bold;"
        "padding-left:94px'>"
        "<span style='width:95px'>τ rated (kg·cm)</span>"
        "<span style='width:75px'>mass (g)</span></div>"
    )
    servo_rows = widgets.VBox([
        servo_header,
        widgets.HBox([
            widgets.Label("s_L2  hip", layout=_lbl_layout),
            nt_trq_h,
            nt_m_h,
        ]),
        widgets.HBox([
            widgets.Label("s_L3  knee", layout=_lbl_layout),
            nt_trq_k,
            nt_m_k,
        ]),
        widgets.HBox([
            widgets.Label("s_L1  shoulder", layout=_lbl_layout),
            nt_trq_sh,
            nt_m_sh,
        ]),
    ])

    sliders_box = widgets.VBox(
        [
            widgets.HTML("<b>Pitch axis parameters</b>"),
            sl_r2,
            sl_r3,
            sl_theta_hip,
            sl_theta_kne,
            widgets.HTML(
                "<hr style='margin:8px 0'><b>Yaw motion profile</b>"
                "<span style='font-size:11px;color:#888;margin-left:6px'>"
                "shoulder swing | rotational inertia only, no gravity load"
                "</span>"
            ),
            sl_ramp,
            widgets.HTML("<hr style='margin:8px 0'><b>Servo parameters</b>"),
            servo_rows,
        ],
        layout=widgets.Layout(width="380px", padding="10px"),
    )

    table_out = widgets.Output()
    fig_out = widgets.Output()

    def update(change=None):
        r2_v = sl_r2.value
        r3_v = sl_r3.value
        theta_hip_v = sl_theta_hip.value
        theta_kne_v = sl_theta_kne.value
        cos_hip = math.cos(math.radians(theta_hip_v))
        cos_kne = math.cos(math.radians(theta_kne_v))
        ramp_v = sl_ramp.value
        r1_v = get_config("links.L1.length_cm", 8.0)

        m_L1_v = link_mass_kg("L1")
        m_L2_v = link_mass_kg("L2")
        m_L3_v = link_mass_kg("L3")
        m_sh = nt_m_h.value / 1000.0  # hip servo mass (g → kg)
        m_kn = nt_m_k.value / 1000.0  # knee servo mass (g → kg)
        rated_k = nt_trq_k.value  # knee rated torque (kg·cm)
        rated_h = nt_trq_h.value  # hip rated torque (kg·cm)
        rated_y = nt_trq_sh.value  # shoulder rated torque (kg·cm)

        # Inverse solve: max tip load each motor can support
        # Clamp each cosine: angle > 90° means gravity assists → motor unconstrained by tip load
        _MAX_TIP = (
            50.0  # kg — display cap; signals "effectively unlimited at this angle"
        )
        cos_hip_g = max(cos_hip, 0.0)
        cos_kne_g = max(cos_kne, 0.0)

        if cos_kne_g > 1e-6:
            max_tip_knee = (rated_k / cos_kne_g - m_L3_v * r3_v / 2.0) / r3_v
        else:
            max_tip_knee = _MAX_TIP

        hip_num = (
            rated_h
            - (m_L2_v * r2_v / 2.0 + m_kn * r2_v) * cos_hip_g
            - m_L3_v * (r2_v * cos_hip_g + r3_v / 2.0 * cos_kne_g)
        )
        hip_den = r2_v * cos_hip_g + r3_v * cos_kne_g
        if hip_den > 1e-6:
            max_tip_hip = hip_num / hip_den
        else:
            max_tip_hip = _MAX_TIP

        overloaded = min(max_tip_knee, max_tip_hip) < 0
        max_tip = max(min(max_tip_knee, max_tip_hip), 0.0)
        max_tip = min(max_tip, _MAX_TIP)
        max_body_3 = max_tip * 3
        max_body_4 = max_tip * 4
        capped = max_tip >= _MAX_TIP
        limiting = "s_L3 knee" if max_tip_knee <= max_tip_hip else "s_L2 hip"

        # Forward pass at max_tip — for capacity bars
        tau_k_at = torque_knee(r3_v, theta_kne_v, m_L3_v, max_tip)
        tau_h_at = torque_hip(
            r2_v,
            r3_v,
            theta_hip_v,
            theta_kne_v,
            m_L2_v,
            m_L3_v,
            m_kn,
            max_tip,
        )
        # NOTE: Yaw inertia uses fully-extended (straight) leg geometry — an
        # intentional worst-case bound.  When the leg is bent the actual horizontal
        # radii decrease and the true inertia is lower.  The yaw motor has large
        # safety margin, so this conservative approach is acceptable.
        I_v = moment_of_inertia_leg(
            r1_v, r2_v, r3_v, m_L1_v, m_L2_v, m_L3_v, m_sh, m_kn, max_tip
        )
        tau_y_at = torque_yaw(I_v, 60.0, ramp_v)

        def bar_html(tau, rated):
            pct = tau / rated * 100.0 if rated > 0 else 0.0
            pct = max(
                0.0, min(pct, 100.0)
            )  # clamp: negative tau = gravity assists, show 0%
            color = "#27ae60" if pct < 90 else ("#f39c12" if pct <= 100 else "#c0392b")
            return (
                f'<div style="display:inline-flex;align-items:center;gap:6px">'
                f'<div style="background:#e0e0e0;border-radius:4px;width:120px;height:10px;overflow:hidden">'
                f'<div style="background:{color};width:{pct:.0f}%;height:10px"></div></div>'
                f'<span style="font-size:11px;color:#555">{pct:.0f}%</span></div>'
            )

        if overloaded:
            header_html = (
                '<div style="background:#fdecea;border:1px solid #e74c3c;border-radius:4px;'
                'padding:10px 14px;font-size:13px;color:#c0392b">'
                "<b>&#9888; Overloaded</b> — link masses alone exceed servo rating at this angle.<br>"
                "Increase hip or knee angle (bend the leg) to reduce the gravity moment."
                "</div>"
            )
        else:
            tip_str = f"≥ {_MAX_TIP:.0f} kg" if capped else f"{max_tip:.3f} kg"
            b3 = f"≥ {max_body_3:.1f} kg" if capped else f"{max_body_3:.2f} kg"
            b4 = f"≥ {max_body_4:.1f} kg" if capped else f"{max_body_4:.2f} kg"
            header_html = (
                f'<div style="padding:10px 14px;background:#eaf6ff;border-radius:4px;border:1px solid #aed6f1">'
                f'<div style="font-size:18px;font-weight:bold;margin-bottom:4px">'
                f"Max supported body weight: &nbsp;"
                f"3-leg: {b3} &nbsp;|&nbsp; 4-leg: {b4}"
                f"</div>"
                f'<div style="font-size:11px;color:#888">per-leg tip load: {tip_str} &nbsp;·&nbsp; limiting motor: {limiting}</div>'
                f"</div>"
            )

        rows = [
            ("s_L3  knee (pitch)", tau_k_at, rated_k),
            ("s_L2  hip (pitch)", tau_h_at, rated_h),
            ("s_L1  shoulder (yaw, dyn)", tau_y_at, rated_y),
        ]
        row_html = ""
        for i, (label, tau, rated) in enumerate(rows):
            bg = " style='background:#f9f9f9'" if i % 2 else ""
            row_html += (
                f"<tr{bg}>"
                f"<td style='padding:5px 10px;text-align:left'>{label}</td>"
                f"<td style='padding:5px 10px;text-align:center'>{tau:.3f}</td>"
                f"<td style='padding:5px 10px;text-align:center'>{rated:.1f}</td>"
                f"<td style='padding:5px 10px;text-align:center'>{bar_html(tau, rated)}</td>"
                f"</tr>"
            )

        table_html = (
            f'<table style="border-collapse:collapse;font-size:12px;min-width:420px">'
            f'<tr style="background:#f0f0f0">'
            f'<th style="padding:5px 10px;text-align:left">Motor</th>'
            f'<th style="padding:5px 10px;text-align:center">τ used (kg·cm)</th>'
            f'<th style="padding:5px 10px;text-align:center">τ rated</th>'
            f'<th style="padding:5px 10px;text-align:center">Capacity</th>'
            f"</tr>"
            f"{row_html}"
            f"</table>"
            f'<p style="font-size:11px;color:#888;margin-top:6px">'
            f"hip={theta_hip_v:.0f}° knee={theta_kne_v:.0f}° from horizontal &nbsp;|&nbsp; worst case at 0° &nbsp;|&nbsp;"
            f" yaw: 60°/s in {ramp_v:.2f}s</p>"
        )

        with table_out:
            table_out.clear_output(wait=True)
            display(widgets.HTML(header_html + table_html))

        with fig_out:
            fig_out.clear_output(wait=True)
            theta_hip_rad = math.radians(theta_hip_v)
            theta_kne_rad = math.radians(theta_kne_v)

            x0, y0 = 0.0, 0.0
            x1 = x0 + r1_v  # L1 fixed horizontal
            y1 = y0
            x2 = x1 + r2_v * math.cos(theta_hip_rad)
            y2 = y1 - r2_v * math.sin(theta_hip_rad)
            x3 = x2 + r3_v * math.cos(theta_kne_rad)
            y3 = y2 - r3_v * math.sin(theta_kne_rad)

            # Fixed viewport: sized for worst-case reach so scale is stable across angles
            max_x = r1_v + r2_v + r3_v  # fully extended horizontal
            max_y = r2_v + r3_v  # fully bent downward
            pad = 2.0
            span_fixed = max(max_x, max_y) + 2 * pad
            cx_fixed = max_x / 2
            cy_fixed = -max_y / 2

            fig, ax = plt.subplots(figsize=(FIGURE_SIZE_IN, FIGURE_SIZE_IN))

            ax.plot(
                [x0, x1],
                [y0, y1],
                lw=8,
                color="#e67e22",
                solid_capstyle="round",
                label=f"L1 = {r1_v:.1f} cm (fixed)",
            )
            ax.plot(
                [x1, x2],
                [y1, y2],
                lw=8,
                color="#3498db",
                solid_capstyle="round",
                label=f"L2 = {r2_v:.1f} cm",
            )
            ax.plot(
                [x2, x3],
                [y2, y3],
                lw=8,
                color="#9b59b6",
                solid_capstyle="round",
                label=f"L3 = {r3_v:.1f} cm",
            )

            for px, py, name, tau_str, col in [
                (x0, y0, "s_L1 (yaw)", f"{tau_y_at:.3f}", "#7f8c8d"),
                (x1, y1, "s_L2 (hip)", f"{tau_h_at:.2f}", "#e74c3c"),
                (x2, y2, "s_L3 (knee)", f"{tau_k_at:.2f}", "#e74c3c"),
            ]:
                ax.plot(px, py, "o", ms=13, color=col, zorder=5)
                ax.annotate(
                    f"{name}\n{tau_str} kg·cm",
                    (px, py),
                    textcoords="offset points",
                    xytext=(0, 18),
                    ha="center",
                    fontsize=8,
                )

            ax.plot(x3, y3, "^", ms=11, color="#2ecc71", zorder=5)
            ax.annotate(
                "tip",
                (x3, y3),
                textcoords="offset points",
                xytext=(0, 12),
                ha="center",
                fontsize=8,
            )

            ax.legend(loc="upper right", fontsize=8)
            ax.set_xlim(cx_fixed - span_fixed / 2, cx_fixed + span_fixed / 2)
            ax.set_ylim(cy_fixed - span_fixed / 2, cy_fixed + span_fixed / 2)
            grid_step = 5
            ax.set_xticks(
                list(
                    range(
                        math.ceil((cx_fixed - span_fixed / 2) / grid_step) * grid_step,
                        math.floor((cx_fixed + span_fixed / 2) / grid_step) * grid_step
                        + grid_step,
                        grid_step,
                    )
                )
            )
            ax.set_yticks(
                list(
                    range(
                        math.ceil((cy_fixed - span_fixed / 2) / grid_step) * grid_step,
                        math.floor((cy_fixed + span_fixed / 2) / grid_step) * grid_step
                        + grid_step,
                        grid_step,
                    )
                )
            )
            ax.grid(True, color="#e8e8e8", linewidth=0.7, zorder=0)
            ax.tick_params(labelbottom=False, labelleft=False, length=0)
            for spine in ax.spines.values():
                spine.set_visible(False)
            ax.set_title(
                f"Leg: hip={theta_hip_v:.0f}°  knee={theta_kne_v:.0f}°   "
                f"(L1={r1_v} · L2={r2_v} · L3={r3_v} cm)",
                fontsize=10,
            )
            plt.tight_layout()
            plt.show()

    for w in [
        sl_r2,
        sl_r3,
        sl_theta_hip,
        sl_theta_kne,
        sl_ramp,
        nt_trq_h,
        nt_trq_k,
        nt_trq_sh,
        nt_m_h,
        nt_m_k,
        nt_m_sh,
    ]:
        w.observe(update, names="value")
    update()

    top = widgets.HBox(
        [sliders_box, table_out], layout=widgets.Layout(align_items="flex-start")
    )
    fig_centered = widgets.HBox(
        [fig_out], layout=widgets.Layout(justify_content="center")
    )
    return widgets.VBox([top, fig_centered])


display(build_widget())
