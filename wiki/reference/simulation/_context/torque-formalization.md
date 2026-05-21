<!--
WIKI-MIGRATION REFERENCE COPY
Original path:  doc/torque-analysis/torque_formalization.md
Original kind:  spec
Copied on:      2026-05-21
Status:         unreviewed
Maps to:        reference/simulation/torque-analysis.md
-->

> **Reference material.** Verbatim copy of `doc/torque-analysis/torque_formalization.md` kept in `_context/`
> as source for the published wiki page. Edit the parent wiki page, not
> this file. The original at `doc/torque-analysis/torque_formalization.md` is still canonical; this file is
> scaffolding the user will delete manually once the wiki pages exist.
<!--
CONSISTENCY-CHECK 2026-05-21
Verified against: feat/wiki-setup @ 0a1a138
- [ok]    τ_knee (§3.1) and τ_hip (§3.2, independent hip/knee cosines) match torque_calculations.py torque_knee()/torque_hip(); I_leg (§3.3.1) matches moment_of_inertia_leg() incl. parallel-axis I=(1/12)mL²+md².
- [ok]    Inverse solver guards (§4.1/4.2, clamped cos for inverse, unclamped for forward) match the update() closure logic.
- [drift] §5 Code References line numbers off by ~1: torque_knee L147 (doc says 146), torque_hip L186 (185), moment_of_inertia_leg L280 (279), torque_yaw L323 (319), update() L521 (doc 503). Re-anchor on function names in Phase B.
- [drift] Link lengths/masses are NOT in code/simulation/facehugger_config.yaml — they live in doc/torque-analysis/config.yaml (L1/L2=8cm, L3=6cm, PLA ρ=1.24, +15% margin). The sim yaml has no link dims; servo there is DFRobot SER0038 (60g, 2.94 N·m). Torque config servo is DSS-M15S (72g, 15 kg·cm). Different configs — don't conflate.
- [drift] Material: §2.3/derivations imply PLA (config.yaml comment + .py L178 "PLA"), but .py L381 narrative says leg is "PETG". Source self-inconsistent; flag for Phase B.
- [todo]  tip.mass_kg = 0.0 in config.yaml (foot not yet modelled) — capacity numbers are link/servo-only; note in published page.
-->
# Torque Analysis Formalization — Quadruped Leg

## 1. Problem Statement

Given a 3-link serial-chain quadruped leg with known geometry, link masses, and servo torque ratings, determine the **maximum body weight** the robot can support statically.

Each leg carries a fraction of the body weight as a point load at the tip (foot contact point). The binding constraint is whichever servo motor saturates first. Body weight equals the per-leg tip load multiplied by the number of legs in stance phase.

A secondary calculation determines the **dynamic torque** required at the shoulder yaw axis to accelerate the leg during swing phase.

## 2. Model Description

### 2.1 Serial chain

The leg is a 3-link serial chain rotating in the sagittal (pitch) plane:

| Link | From | To | Length | Notation |
|------|------|----|--------|----------|
| $L_1$ | shoulder | hip joint | $r_1$ | fixed horizontal |
| $L_2$ | hip joint | knee joint | $r_2$ | rotates by $\theta_\text{hip}$ |
| $L_3$ | knee joint | tip | $r_3$ | rotates by $\theta_\text{knee}$ |

### 2.2 Axis classification

| Motor | Axis | Load type |
|-------|------|-----------|
| $s_{L2}$ (hip) | Pitch | Static gravity torque |
| $s_{L3}$ (knee) | Pitch | Static gravity torque |
| $s_{L1}$ (shoulder) | Yaw (vertical) | Dynamic rotational inertia |

The yaw axis is parallel to gravity, so it bears zero static torque.

### 2.3 Assumptions

- Links are **uniform-density rigid rods** (centre of mass at midpoint).
- Servos are **point masses** located at the joints they actuate.
- Angles $\theta_\text{hip}$ and $\theta_\text{knee}$ are measured from horizontal; $0°$ is worst case (fully extended).
- The tip load $m_\text{tip}$ acts as a point mass at the end of $L_3$.
- For yaw inertia, the leg is modelled in its **fully extended (straight) configuration** as a conservative worst-case bound (Path B).

## 3. Derivations

### 3.1 Knee motor — $s_{L3}$ (pitch, static)

The knee motor supports $L_3$ and the tip load. All masses lie along $L_3$, so a single angle $\theta_\text{knee}$ governs the projection:

$$\tau_{L3} = \left( m_{L3} \cdot \frac{r_3}{2} + m_\text{tip} \cdot r_3 \right) \cos\theta_\text{knee}$$

### 3.2 Hip motor — $s_{L2}$ (pitch, static)

The hip motor supports $L_2$, the knee servo, $L_3$, and the tip. Masses on $L_2$ are projected by $\cos\theta_\text{hip}$; masses on $L_3$ are projected by $\cos\theta_\text{knee}$:

$$\tau_{L2} = m_{L2} \cdot \frac{r_2}{2} \cdot \cos\theta_\text{hip} + m_\text{servo,knee} \cdot r_2 \cdot \cos\theta_\text{hip} + m_{L3} \left( r_2 \cos\theta_\text{hip} + \frac{r_3}{2} \cos\theta_\text{knee} \right) + m_\text{tip} \left( r_2 \cos\theta_\text{hip} + r_3 \cos\theta_\text{knee} \right)$$

Note: the hip and knee angles are **independent**. The previous (buggy) formulation used a single shared angle, which is only valid when $\theta_\text{hip} = \theta_\text{knee}$ (collinear links).

### 3.3 Shoulder motor — $s_{L1}$ (yaw, dynamic)

#### 3.3.1 Moment of inertia (parallel axis theorem)

Each link is a uniform rod of length $L$ with mass $m$, whose centre of mass is at distance $d$ from the shoulder yaw axis. By the parallel axis theorem:

$$I_\text{link} = \frac{1}{12} m L^2 + m d^2$$

For point masses (servos, tip):

$$I_\text{point} = m \cdot d^2$$

Combining all components (fully extended leg):

$$I_\text{leg} = \sum_{\text{links}} \left( \frac{1}{12} m_i L_i^2 + m_i d_i^2 \right) + \sum_{\text{points}} m_j d_j^2$$

Specifically:

| Component | $m$ | $L$ | $d$ (CoM distance from shoulder) |
|-----------|-----|-----|----------------------------------|
| $L_1$ | $m_{L1}$ | $r_1$ | $r_1/2$ |
| $L_2$ | $m_{L2}$ | $r_2$ | $r_1 + r_2/2$ |
| $L_3$ | $m_{L3}$ | $r_3$ | $r_1 + r_2 + r_3/2$ |
| Hip servo | $m_\text{servo,hip}$ | — | $r_1$ |
| Knee servo | $m_\text{servo,knee}$ | — | $r_1 + r_2$ |
| Tip | $m_\text{tip}$ | — | $r_1 + r_2 + r_3$ |

**Conservative bound (Path B):** The distances $d$ above assume a fully extended leg. When the leg is bent, horizontal radii decrease and $I$ is lower. Using straight-leg geometry provides a worst-case upper bound on required yaw torque.

#### 3.3.2 Dynamic torque

$$\tau_\text{yaw} = I_\text{leg} \cdot \alpha$$

where $\alpha = \Delta\omega / \Delta t$ is the angular acceleration in rad/s$^2$.

## 4. Inverse Solver

### 4.1 Knee — max tip load

Set $\tau_{L3} = \tau_\text{rated,knee}$ and solve for $m_\text{tip}$:

$$m_\text{tip,max}^\text{knee} = \frac{\tau_\text{rated,knee} / \cos\theta_\text{knee} - m_{L3} \cdot r_3/2}{r_3}$$

**Guard:** If $\cos\theta_\text{knee} \leq 0$ (angle $\geq 90°$), gravity assists the motor and the knee is unconstrained. Report as unlimited.

### 4.2 Hip — max tip load

Set $\tau_{L2} = \tau_\text{rated,hip}$ and solve for $m_\text{tip}$:

$$m_\text{tip,max}^\text{hip} = \frac{\tau_\text{rated,hip} - \left( m_{L2} \cdot r_2/2 + m_\text{servo,knee} \cdot r_2 \right) \cos\theta_\text{hip} - m_{L3} \left( r_2 \cos\theta_\text{hip} + r_3/2 \cdot \cos\theta_\text{knee} \right)}{r_2 \cos\theta_\text{hip} + r_3 \cos\theta_\text{knee}}$$

**Guard:** If the denominator $r_2 \cos\theta_\text{hip} + r_3 \cos\theta_\text{knee} < \epsilon$, both cosines are near zero and the motor is unconstrained. Report as unlimited.

Note: The clamped cosines $\cos_g = \max(\cos\theta, 0)$ are used in the inverse solver to avoid division by zero when angles exceed 90°. The **forward** torque computation uses unclamped cosines (negative values represent gravity assist and are physically meaningful).

### 4.3 Binding constraint

$$m_\text{tip,max} = \min\left( m_\text{tip,max}^\text{knee},\; m_\text{tip,max}^\text{hip} \right)$$

$$m_\text{body} = m_\text{tip,max} \times n_\text{legs in stance}$$

## 5. Code References

All implementations are in `torque_calculations.py`:

| Formula | Function | Line |
|---------|----------|------|
| Knee torque $\tau_{L3}$ | `torque_knee()` | 146 |
| Hip torque $\tau_{L2}$ | `torque_hip()` | 185 |
| Moment of inertia $I_\text{leg}$ | `moment_of_inertia_leg()` | 279 |
| Yaw dynamic torque $\tau_\text{yaw}$ | `torque_yaw()` | 319 |
| Inverse solver + widget | `build_widget() > update()` | 503 |

## 6. Units and Conversions

| Quantity | Unit | Notes |
|----------|------|-------|
| Link lengths | cm | Config values |
| Masses | kg | Computed from volume and density |
| Torque (servo ratings) | kg·cm | Industry convention for hobby servos |
| Torque (SI) | N·m | $1\;\text{kg·cm} = 0.0981\;\text{N·m}$ |
| Conversion factor | — | $1\;\text{N·m} = 10.197\;\text{kg·cm}$ |
| Moment of inertia | kg·cm² | Internal computation |
| Moment of inertia (SI) | kg·m² | $1\;\text{kg·cm}^2 = 10^{-4}\;\text{kg·m}^2$ |

The `torque_yaw()` function converts internally: $I$ in kg·cm² $\to$ kg·m² ($\times 10^{-4}$), computes $\tau$ in N·m, then converts back to kg·cm ($\times 10.197$).
