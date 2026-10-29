# Architecture Decision Record: MkDocs Documentation Site Architecture

- **Status:** Approved & Documented
- **Date:** 2026-10-29
- **Deciders:** Robotics Educational Content Lead & Engineering Pair

---

## 1. Context and Problem Statement

The repository contains extensive, modular educational documentation organized across numbered top-level directories:
- `00-getting-started/`
- `01-module-53-tf2-kinematics/`
- `02-module-54-mathematics/`
- `03-robot-kinematics/`
- `04-python-robotics/`
- `05-ros2-practical/`
- `06-labs-and-projects/`
- `07-assessments/`
- `docs/` (audit, reference centre, visual index, lesson template)

We want to provide an interactive, searchable, and web-hosted documentation site using **MkDocs Material** for GitHub Pages deployment. Standard MkDocs expects documentation Markdown files to live inside a dedicated `docs/` subfolder.

---

## 2. Considered Alternatives

### Option A: Move All Lesson Folders into `docs/`
- **Approach:** Move `00-...`, `01-...`, `02-...`, etc., into `docs/`.
- **Pros:** Default MkDocs convention without custom `docs_dir`.
- **Cons:**
  - **Severe File Churn:** Shifts 70+ files, destroying concise Git blame tracking.
  - **Broken Relative Links:** Breaks existing external links, README pointers, and internal relative links to `assets/` and `eduarm_description/`.
  - **ROS 2 Colcon Friction:** ROS 2 developers expect robot description packages (`eduarm_description`) and practical directories to live at predictable top-level workspace locations.
- **Verdict:** **Rejected** due to unacceptable regression risk and unnecessary churn.

---

### Option B: Build-Time Mirroring / Symlinking
- **Approach:** Write a CI script that copies or symlinks lesson directories into a temporary `docs/` directory during GitHub Actions build.
- **Pros:** Keeps top-level structure clean.
- **Cons:**
  - On Windows machines, creating filesystem symlinks requires Administrator privileges or Windows Developer Mode enabled.
  - Build-time file duplication can lead to accidental edits to build artifacts instead of source markdown files.
- **Verdict:** **Rejected** due to cross-platform developer friction.

---

### Option C (Selected): Place `mkdocs.yml` in `.mkdocs/` with `docs_dir: ".."`
- **Approach:** Place `mkdocs.yml` inside a `.mkdocs/` subdirectory so that `docs_dir: ".."` resolves to the repository root, satisfying MkDocs 1.6.1's requirement that `docs_dir` must not be the direct parent of `mkdocs.yml`. Set `site_dir` to a sibling outside the repository so it is never committed.
- **Pros:**
  - **Zero File Moves:** Every file remains in its natural, top-level git directory.
  - **Zero Link Breakage:** All 250+ relative links and images remain 100% valid.
  - **MkDocs 1.6.1 Compatible:** Satisfies the hard check that `docs_dir != parent(config_file)`.
  - **Dual Usability:** The repository reads seamlessly directly on GitHub (in native Markdown) AND compiles into a static website with search, syntax highlighting, and KaTeX math formatting.
- **Verdict:** **Adopted** as the canonical architecture.

> **Why not `docs_dir: "."`?**
> MkDocs 1.6.1 added a hard validation error:
> `"The 'docs_dir' should not be the parent directory of the config file."`
> This fires whenever `mkdocs.yml` sits in the same directory as `docs_dir`. The `.mkdocs/` subdirectory pattern is the minimal workaround — the config file moves one level down; `docs_dir: ".."` still resolves to the repo root.

---

## 3. Implementation Details

The configuration at `.mkdocs/mkdocs.yml` configures:
1. `docs_dir: ".."` — resolves to the repository root (one level up from `.mkdocs/`).
2. `site_dir: "../../robotics-notes-site"` — sibling to the repo root; never committed; listed in `.gitignore`.
3. `exclude_docs` for `.git/**`, `venv/**`, `.pytest_cache/**`, `scripts/**`, `eduarm_description/**`, Python source files (`04-python-robotics/**/*.py`), workflow and config files, and other non-documentation artifacts.
4. Math rendering via `pymdownx.arithmatex` with MathJax CDN.
5. Mermaid diagram rendering via `pymdownx.superfences.fence_code_format`.
6. Material theme features: navigation tabs, instant search, code block copy button, and dark/light mode toggle.

To build the documentation locally:

```bash
# From the repository root
python -m mkdocs build -f .mkdocs/mkdocs.yml --strict
```

To serve locally with live-reload:

```bash
python -m mkdocs serve -f .mkdocs/mkdocs.yml
```

