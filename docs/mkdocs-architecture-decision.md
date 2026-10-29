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

### Option C (Selected): Set `docs_dir: .` in `mkdocs.yml` with Explicit Navigation
- **Approach:** Configure MkDocs to treat the repository root as the documentation source directory (`docs_dir: .`), filter out non-documentation directories via `exclude_docs`, and define an explicit, ordered `nav` tree matching the pedagogical learning path.
- **Pros:**
  - **Zero File Moves:** Every file remains in its natural, top-level git directory.
  - **Zero Link Breakage:** All 250+ relative links and images remain 100% valid.
  - **Dual Usability:** The repository reads seamlessly directly on GitHub (in native Markdown) AND compiles into a static website with search, syntax highlighting, and KaTeX math formatting.
- **Verdict:** **Adopted** as the canonical architecture.

---

## 3. Implementation Details

The root `mkdocs.yml` configures:
1. `docs_dir: .`
2. `exclude_docs` for `.git/**`, `venv/**`, `.pytest_cache/**`, `scripts/**`, and non-doc build artifacts.
3. Math rendering via `pymdownx.arithmatex` with KaTeX / MathJax JavaScript CDN.
4. Mermaid diagram rendering via `pymdownx.superfences`.
5. Material theme features: navigation tabs, instant search, code block copy button, and dark/light mode toggle.
