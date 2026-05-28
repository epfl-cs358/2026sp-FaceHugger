"""firmware_sil — drive the PyBullet sim with the EXACT firmware control code.

The `fh_sim` pybind11 module (built from code/firmware/src by CMakeLists.txt) runs
the real firmware SpinalCord on the host; `sil_bridge` converts the servo angles it
produces into URDF joint targets and drives PyBullet. See EXACT-FIRMWARE-SIL-PLAN.md.
"""
