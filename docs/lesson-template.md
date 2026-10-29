# Lesson Template & Pedagogical Style Guide

This template defines the standard 10-component structure for theoretical and practical lessons across this repository. When authoring or revising chapters, follow this structure to ensure consistent clarity, technical rigor, and student accessibility.

---

## Standard 10-Component Structure

```markdown
# [Lesson Number]. [Lesson Title]

<!-- COMPONENT 1: METADATA & OBJECTIVES -->
> **Prerequisites:** [List prior chapters or required mathematical concepts]  
> **Estimated Study Time:** [e.g., 30–45 minutes]  
> **Core Learning Outcomes:**
> - [Outcome 1: Understand / derive...]
> - [Outcome 2: Calculate / implement...]
> - [Outcome 3: Verify / debug...]

---

## 1. Intuitive Introduction & Physical Context
<!-- COMPONENT 2: INTUITIVE MOTIVATION & DIAGRAM -->
[Begin with a real-world robotics scenario explaining WHY this concept exists.]
[Include a conceptual diagram or SVG schematic visualizing the coordinate frames, forces, or kinematic chain.]

Diagram syntax: `![Diagram Caption]{../assets/diagrams/<diagram-name>.svg}`

---

## 2. Mathematical Definitions & Notation
<!-- COMPONENT 3: SYMBOLS & SI UNITS TABLE -->
[Formalize the concepts with precise mathematical definitions.]

| Symbol | Mathematical Domain | Physical Meaning | SI Units |
| :--- | :--- | :--- | :--- |
| $\mathbf{p}$ | $\mathbb{R}^3$ | Position vector in Euclidean 3D space | $\text{m}$ |
| $\mathbf{R}$ | $SO(3)$ | Orthogonal rotation matrix | Unitless |
| $\boldsymbol{\omega}$ | $\mathbb{R}^3$ | Angular velocity vector | $\text{rad/s}$ |

---

## 3. Step-by-Step Derivation
<!-- COMPONENT 4: RIGOROUS STEP-BY-STEP DERIVATION -->
[Develop the equations systematically from first principles without skipping steps.]
[Number key formulas and annotate algebraic substitutions.]

$$\mathbf{p}_{\text{new}} = \mathbf{R} \, \mathbf{p}_{\text{orig}} + \mathbf{d}$$

---

## 4. Worked Numerical Example
<!-- COMPONENT 5: WORKED NUMERICAL EXAMPLE -->
[Provide a complete numerical problem with realistic robot dimensions.]
[Show intermediate steps: plug in numbers, calculate intermediate terms, show final result.]

- **Given Parameters:**
  - $a = 0.50\,\text{m}$
  - $\theta = 30^\circ = \frac{\pi}{6}\,\text{rad}$
- **Step 1:** Compute trigonometric terms: $\cos(30^\circ) = 0.8660$, $\sin(30^\circ) = 0.5000$.
- **Step 2:** Formulate transformation matrix...
- **Final Result:** $\mathbf{p} = [0.4330, 0.2500, 0.0000]^T\,\text{m}$.

---

## 5. Python Implementation / ROS 2 Commands
<!-- COMPONENT 6: CODE IMPLEMENTATION -->
[Provide a clear, self-contained Python function or ROS 2 terminal workflow implementing the derivation.]

```python
import numpy as np

def compute_example(theta_rad: float, length_m: float) -> np.ndarray:
    """Docstring explaining inputs, outputs, and units."""
    return length_m * np.array([np.cos(theta_rad), np.sin(theta_rad)])
```

---

## 6. Expected Output & Verification Checkpoint
<!-- COMPONENT 7: EXPECTED OUTPUT & CHECKPOINTS -->
[Show the exact console output produced by the code.]

```text
Expected Output:
Position: [0.433 0.250] m
Norm: 0.500 m
```

> [!NOTE]
> **Self-Check:** Verify that the magnitude equals link length $0.50\,\text{m}$.

---

## 7. Common Pitfalls & Troubleshooting
<!-- COMPONENT 8: COMMON MISTAKES -->
- **Pitfall 1:** Passing degrees instead of radians to numpy functions (`np.sin(30)` instead of `np.sin(np.radians(30))`).
- **Pitfall 2:** Swapping argument order in `atan2(y, x)`.
- **Pitfall 3:** Premature matrix multiplication reversal ($\mathbf{A}\mathbf{B} \neq \mathbf{B}\mathbf{A}$).

---

## 8. Summary & Self-Test Questions
<!-- COMPONENT 9: RECAP & CONCEPTUAL QUESTIONS -->
- **Key Takeaway 1:** ...
- **Key Takeaway 2:** ...

### Quick Self-Test:
1. *Question 1?*
   <details><summary>Click to view solution</summary>Brief solution explanation.</details>
2. *Question 2?*
   <details><summary>Click to view solution</summary>Brief solution explanation.</details>

---

## Navigation
<!-- COMPONENT 10: PREV / NEXT / PARENT LINKS -->
Navigation syntax: `| [← Previous]{<prev-file.md>} | [Module Index]{<README.md>} | [Next →]{<next-file.md>} |`
```
