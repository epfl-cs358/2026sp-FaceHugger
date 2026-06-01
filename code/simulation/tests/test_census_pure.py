"""test_census_pure.py — Pure-Python tests for coordinate helpers in census.py.

Injects a stub ``adsk`` module so the import succeeds without a real Fusion session.

Path setup: repo root is parents[3] from this file.
  this file: code/simulation/tests/test_census_pure.py
  parents[0]: code/simulation/tests/
  parents[1]: code/simulation/
  parents[2]: code/
  parents[3]: repo root (2026sp-FaceHugger/)
"""

import importlib.util
import sys
import types
from pathlib import Path

# ── adsk stub — must be injected BEFORE importing census ─────────────────────

_adsk = types.ModuleType("adsk")
_core = types.ModuleType("adsk.core")
_fusion = types.ModuleType("adsk.fusion")


class _FakeMatrix:
    @classmethod
    def create(cls):
        m = cls()
        # Row-major 4x4 identity: a[i*4+j] = 1 if i==j else 0
        m._arr = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
        return m

    def asArray(self):
        return list(self._arr)


_core.Matrix3D = _FakeMatrix
_fusion.CalculationAccuracy = types.SimpleNamespace(VeryHighCalculationAccuracy=2)
_adsk.core = _core
_adsk.fusion = _fusion
sys.modules.update({"adsk": _adsk, "adsk.core": _core, "adsk.fusion": _fusion})


# Load census by file path to avoid colliding with any other "lib" package
# cached in sys.modules (e.g. from test_config_loader).
_CENSUS_PATH = Path(__file__).parents[3] / "cad/scripts/FusionURDFExport/lib/census.py"
_spec = importlib.util.spec_from_file_location("_fu_census", _CENSUS_PATH)
_census_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_census_mod)

_axis_to_world = _census_mod._axis_to_world
_collect_rigid_groups = _census_mod._collect_rigid_groups
_mat_trans_mm = _census_mod._mat_trans_mm
_pt_mm = _census_mod._pt_mm
_pt_world_mm = _census_mod._pt_world_mm

# ── Fake helpers ──────────────────────────────────────────────────────────────


class _FakePt:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z


class _FakeOcc:
    def __init__(self, rot_3x3):
        m = _FakeMatrix.create()
        r = rot_3x3
        # Row-major 4x4: rotation in top-left 3x3, translation column = 0
        m._arr = [
            r[0][0], r[0][1], r[0][2], 0,
            r[1][0], r[1][1], r[1][2], 0,
            r[2][0], r[2][1], r[2][2], 0,
            0, 0, 0, 1,
        ]  # fmt: skip
        self.transform2 = m


# R_LA: confirmed from Fusion exploration 2026-06-01
# Shoulder axis [0,0,1] → [0,0,-1] world; hip/knee [1,0,0] → [0,1,0] world
R_LA = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]
_OCC_RLA = _FakeOcc(R_LA)


class _FakeRG:
    def __init__(self, name, paths):
        self.name = name
        self.occurrences = [types.SimpleNamespace(fullPathName=p) for p in paths]


class _FakeRoot:
    def __init__(self):
        self.allRigidGroups = [
            _FakeRG("Link1RigidGroup", ["FaceHuggerLegAssembly:1+Link1L:1"])
        ]


# ── Test 1: _pt_mm converts cm → mm ──────────────────────────────────────────


def test_pt_mm_converts_cm_to_mm():
    result = _pt_mm(_FakePt(1.0, 0.0, 0.0))
    assert result == [10.0, 0.0, 0.0]


# ── Test 2: _mat_trans_mm extracts translation ────────────────────────────────


def test_mat_trans_mm_identity_is_zero():
    m = _FakeMatrix.create()
    assert _mat_trans_mm(m) == [0.0, 0.0, 0.0]


def test_mat_trans_mm_with_translation():
    m = _FakeMatrix.create()
    # Translation in cm: (1, 2, 3) → expect mm: (10, 20, 30)
    # Row-major 4x4: translation is a[3], a[7], a[11]
    m._arr = [1, 0, 0, 1, 0, 1, 0, 2, 0, 0, 1, 3, 0, 0, 0, 1]
    assert _mat_trans_mm(m) == [10.0, 20.0, 30.0]


# ── Test 3: _pt_world_mm with identity transform ──────────────────────────────


def test_pt_world_mm_identity():
    m = _FakeMatrix.create()
    pt = _FakePt(2.0, 3.0, 4.0)
    # Identity transform: world = local, then *10 for mm
    assert _pt_world_mm(pt, m) == [20.0, 30.0, 40.0]


# ── Test 4: _pt_world_mm with translation ────────────────────────────────────


def test_pt_world_mm_with_translation():
    m = _FakeMatrix.create()
    # Translate by (1, 0, 0) cm → point (0,0,0) shifts to (1,0,0) cm = (10,0,0) mm
    m._arr = [1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
    pt = _FakePt(0.0, 0.0, 0.0)
    result = _pt_world_mm(pt, m)
    assert result == [10.0, 0.0, 0.0]


# ── Test 5: _axis_to_world — shoulder axis [0,0,1] with R_LA → [0,0,-1] ──────


def test_axis_to_world_shoulder():
    result = _axis_to_world([0, 0, 1], _OCC_RLA)
    assert result == [0.0, 0.0, -1.0]


# ── Test 6: _axis_to_world — hip/knee axis [1,0,0] with R_LA → [0,1,0] ──────


def test_axis_to_world_hip_knee():
    result = _axis_to_world([1, 0, 0], _OCC_RLA)
    assert result == [0.0, 1.0, 0.0]


# ── Test 7: _axis_to_world — no parent occurrence → axis unchanged ─────────


def test_axis_to_world_no_parent():
    result = _axis_to_world([1, 0, 0], None)
    assert result == [1.0, 0.0, 0.0]


# ── Test 8: _collect_rigid_groups with one fake rigid group ───────────────────


def test_collect_rigid_groups_single_group():
    root = _FakeRoot()
    result = _collect_rigid_groups(root)
    assert len(result) == 1
    assert result[0]["name"] == "Link1RigidGroup"
    assert result[0]["occurrence_paths"] == ["FaceHuggerLegAssembly:1+Link1L:1"]
