# Torque analysis

!!! todo "Stub - to be written"
    Full torque model. Narrative adapted from `_context/torque-formalization.md`.
    Embed static result images via glightbox; link the notebook for interactive
    use. The accessible summary lives in
    [Design → Sizing](../../guide/design/sizing.md).

## The model

!!! todo
    Link lengths, masses, gravity loading, the static support problem.

## Key equations

!!! todo
    \[
    \tau_{hip} = \left(\frac{m_{L2} L_2}{2} + m_{servo} L_2
                 + m_{L3}\left(L_2 + \frac{L_3}{2}\right)
                 + F_{tip}(L_2 + L_3)\right)\cos\theta_{hip}
    \]

## Results

!!! todo
    Static result plots (glightbox).

!!! note "Interactive notebook"
    An interactive version with sliders for link lengths and servo angles is
    available as a Jupyter notebook at
    `doc/torque-analysis/torque_calculations.ipynb` - run it locally for live
    interaction (the static site has no Python kernel).
