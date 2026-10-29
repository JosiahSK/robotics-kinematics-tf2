# Contributing to Robotics & AI Student Notes

Welcome! We are excited that you want to contribute to this open-source educational repository for robotics and artificial intelligence.

## Table of Contents
1. [Code of Conduct](#code-of-conduct)
2. [How Can I Contribute?](#how-can-i-contribute)
3. [Mathematical & Code Standards](#mathematical--code-standards)
4. [Development Workflow](#development-workflow)
5. [Reporting Bugs & Requesting Features](#reporting-bugs--requesting-features)

---

## Code of Conduct

We are committed to providing a welcoming, inclusive, and harassment-free environment for everyone regardless of level of experience, gender, identity, sexual orientation, disability, appearance, race, or religion.

---

## How Can I Contribute?

- **Fixing Typos & Clarifying Explanations:** If an explanation or derivation feels confusing, submit an improvement.
- **Adding Worked Examples:** Practical, step-by-step engineering examples help students bridge theory and real hardware.
- **Improving Unit Tests:** Extend edge case coverage in `04-python-robotics/tests/`.
- **Expanding ROS 2 Nodes:** Add launch files, URDF snippets, or TF2 broadcaster examples compatible with ROS 2 Jazzy (primary LTS) or Humble (legacy LTS).

---

## Mathematical & Code Standards

### Mathematical Rigour
1. **Equations:** Use LaTeX syntax (`$...$` for inline math, `$$...$$` for display blocks).
2. **Definitions:** Define every symbol, vector, matrix, and coordinate frame immediately upon introduction.
3. **Frames:** Explicitly state the reference frame and the target frame using standard notation (e.g., ${}^A T_B$ or $T_A^B$).
4. **Units:** Always state physical units (SI units: metres $\text{m}$, radians $\text{rad}$, seconds $\text{s}$).

### Python Code Quality
- Follow PEP 8 guidelines.
- Include explicit type hints (`numpy.ndarray`, `float`, `Tuple[...]`).
- Write docstrings adhering to NumPy or Google style.
- All algorithms must be pure NumPy where possible to avoid opaque black-box dependencies.
- Ensure all tests pass:
  ```bash
  pytest 04-python-robotics/tests/
  ```

---

## Development Workflow

1. Fork the repository on GitHub.
2. Clone your fork locally:
   ```bash
   git clone https://github.com/<your-username>/robotics-kinematics-tf2.git
   cd robotics-kinematics-tf2
   ```
3. Create a descriptive branch:
   ```bash
   git checkout -b feature/analytical-ik-spherical-wrist
   ```
4. Set up your virtual environment and install dependencies:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: .\venv\Scripts\Activate.ps1
   pip install -r requirements.txt
   ```
5. Implement your changes, write tests, and run:
   ```bash
   pytest
   ```
6. Commit your changes with clear, semantic commit messages:
   ```bash
   git commit -m "docs(kinematics): add singularity ellipsoid derivation"
   ```
7. Push to your fork and submit a Pull Request!

Thank you for helping empower the next generation of roboticists!
