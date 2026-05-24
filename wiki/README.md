# Working on the FaceHugger wiki

This folder is the source of the project wiki (a [MkDocs Material](https://squidfunk.github.io/mkdocs-material/)
site). This page explains how the wiki is built and how to contribute to it. It is **not** published to the
site itself; it's here for editors.

## How the wiki works

- **Source:** every page is a Markdown file under `wiki/`. The config is `mkdocs.yml` at the repo root.
- **Two pillars:** `wiki/guide/` is the build guide (for builders/newcomers); `wiki/reference/` is the
  technical depth. Each is a top tab on the site.
- **Navigation** comes from a `.pages` file in each folder (it sets the section's title and page order).
  There is no hand-maintained nav list. Add a page by creating the `.md` and adding its filename to that
  folder's `.pages`.
- **`_context/` folders** hold the original project docs copied verbatim, as source material for writing the
  real pages. They are excluded from the built site but stay in the repo so you can read them on GitHub.
  **Read the `CONSISTENCY-CHECK` note at the top of each `_context/` file first** - many describe designs
  that changed or were never built. Adapt the content, don't paste it. Don't edit `_context/` files; they get
  deleted once the corresponding page is written.
- **Publishing:** a push to `main` touching `wiki/**` runs `.github/workflows/deploy-docs.yml`, which builds
  with `mkdocs build --strict` and deploys to GitHub Pages. A broken link fails the build.

## Working on this branch

We write the wiki on the **`feat/wiki-setup`** branch. The live site only updates when this is merged to
`main`, so until then, **preview locally**.

```bash
git clone https://github.com/epfl-cs358/2026sp-FaceHugger.git
cd 2026sp-FaceHugger
git checkout feat/wiki-setup

# one-time: docs toolchain in an isolated venv
python -m venv .venv-docs
source .venv-docs/bin/activate
pip install -r requirements-docs.txt

# live preview at http://127.0.0.1:8000 (rebuilds as you save)
source .venv-docs/bin/activate && mkdocs serve
```

Edit your pages, then:

```bash
git add wiki/...
git commit -m "docs(wiki): write <page>"
git push                       # pushes to feat/wiki-setup
```

Pull often (`git pull`) so you stay in sync with teammates on the same branch.

## Writing a page

- Pages are stubs with `!!! todo` markers - replace them with real content.
- Keep code fences language-tagged (` ```cpp `, ` ```python `, ` ```yaml `).
- Use admonitions for callouts: `!!! note`, `!!! warning`, `!!! tip`, `!!! danger`.
- Cross-link other pages with relative links so they survive the build's link check.

## Adding images, GIFs, video, 3D models & notebooks

Any non-Markdown file under `wiki/` is copied to the site as-is. Convention: an `img/` folder next to the
page that uses it; shared assets in `wiki/assets/`. Reference with a path relative to the page.

**Images & GIFs:** a GIF is just an image; it animates on its own. Click-to-zoom is automatic (glightbox).

```markdown
![Assembled robot](img/robot.jpg){ width="500" }

<figure markdown="span">
  ![Leg detail](img/leg.png){ width="300" }
  <figcaption>Knee joint, exploded view</figcaption>
</figure>
```

**Video:** for small clips, put an `.mp4` in `img/` with an HTML5 tag. For large videos, use a YouTube embed instead of
committing big binaries to git.

```html
<video controls width="100%"><source src="img/walk.mp4" type="video/mp4"></video>
```

**3D models (STL viewer):** the `<model-viewer>` web component shows interactive 3D, but it loads `.glb`,
not `.stl`. Convert once with [`assets/models/convert_stl_to_glb.py`](assets/models/convert_stl_to_glb.py),
commit the `.glb`, then on the page:

```html
<script type="module" src="https://unpkg.com/@google/model-viewer/dist/model-viewer.min.js"></script>

<model-viewer src="../assets/models/QuadrupedBody.glb" camera-controls auto-rotate
              style="width:100%;height:400px;background:#1a1a2e;"></model-viewer>
```

No-conversion fallback: an iframe to `viewstl.com` pointed at the STL's raw-GitHub URL.

**Interactive notebooks:** `mkdocs-jupyter` is enabled. Drop a `.ipynb` under `wiki/` and add it to the
folder's `.pages`; it renders with its **saved** outputs. The site has no Python kernel, so `ipywidgets`
sliders aren't live. For the torque analysis: ship the notebook read-only and tell readers to run it locally,
or reimplement the controls as a self-contained Plotly HTML widget embedded in an `<iframe>`.

**Math & diagrams:** LaTeX via MathJax: inline `\( ... \)`, block `\[ ... \]`. Diagrams via Mermaid: a
` ```mermaid ` fenced block. Circuit schematics: export from KiCad as SVG and embed as an image.

## Who writes what

Starting split by theme; reassign names and rebalance per page as you go (this is WIP). Ferdinand's
firmware+software block is the heaviest; consider sharing the firmware reference.

| Member | Theme | Pages to write |
|---|---|---|
| **Antoine V.** | Design & analysis | `wiki/index.md` (Quick Start); `guide/design.md` (concept, how a leg moves, sizing); `reference/simulation/` (torque analysis, torque heatmap) |
| **Antoine R.** | Parts & 3D printing | `guide/parts.md` (BOM, electronics, hardware & ball bearings, printed parts); `guide/printing.md` |
| **Ilias S.** | Wiring & assembly | `guide/wiring.md` (schematic, pinout, battery safety); `guide/assembly.md` (step-by-step build + photos) |
| **Ferdinand C.** | Firmware & software | `guide/software.md` (overview, how-it-works, setup, running); `reference/firmware/` (architecture, kinematics, servo conventions, API, CSV diagnostics) |
| **Noa D.** | Animation & app | `reference/animation/` (pipeline, Blender rig, URDF pipeline, `.fhc` format, gait design); `reference/remote-control/` (app overview, WebSocket API) |

Full page list and the source-doc mapping: see `WIKI_MIGRATION_PLAN.md` §7 at the repo root.
