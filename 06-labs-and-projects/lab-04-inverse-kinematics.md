# Lab 04: Planar Inverse Kinematics & Workspace Reachability

> **Prerequisites:** [07: Inverse Kinematics Fundamentals](../03-robot-kinematics/07-inverse-kinematics.md), [06: Forward Kinematics](../03-robot-kinematics/06-forward-kinematics.md), Python 3.10+.  
> **Estimated Time:** 45–60 minutes  
> **Code Files:**
> - Starter / Reference: [`04-python-robotics/inverse_kinematics_2d.py`](../04-python-robotics/inverse_kinematics_2d.py)
> - Unit Test Suite: [`04-python-robotics/tests/test_ik.py`](../04-python-robotics/tests/test_ik.py)

---

## 1. Laboratory Objectives
In this laboratory, you will:
1. Implement the analytical Law of Cosines inverse kinematics algorithm for a 2-link planar arm.
2. Program logic that computes both **Elbow-Up** and **Elbow-Down** solutions.
3. Detect unreachable targets (both outer radius $r > l_1 + l_2$ and inner deadzone $r < |l_1 - l_2|$).
4. Verify computed joint angles by piping them back through forward kinematics to confirm position errors $< 10^{-6}\,\text{m}$.

---

## 2. Mathematical Algorithm
Given link lengths $l_1, l_2$ and target Cartesian coordinates $(x, y)$:
1. **Target Distance:** $r = \sqrt{x^2 + y^2}$.
2. **Reachability Check:** If $r > l_1 + l_2$ or $r < |l_1 - l_2|$, return `None` (unreachable).
3. **Elbow Angle $\theta_2$ (Law of Cosines):**
   $$\cos\theta_2 = \frac{x^2 + y^2 - l_1^2 - l_2^2}{2 l_1 l_2}$$
   - **Elbow-Down:** $\theta_2 = +\arccos(\cos\theta_2)$
   - **Elbow-Up:** $\theta_2 = -\arccos(\cos\theta_2)$
4. **Shoulder Angle $\theta_1$:**
   $$\theta_1 = \operatorname{atan2}(y, x) - \operatorname{atan2}\left(l_2 \sin\theta_2, \; l_1 + l_2 \cos\theta_2\right)$$

---

## 3. Step-by-Step Task Sequence & Run Commands

### Task 1: Execute the Planar IK Reference Engine
Run [`04-python-robotics/inverse_kinematics_2d.py`](../04-python-robotics/inverse_kinematics_2d.py):
```bash
python 04-python-robotics/inverse_kinematics_2d.py
```

#### Expected Output for $l_1 = 1.0\,\text{m}, l_2 = 0.8\,\text{m}$, Target $(1.20, 0.60)\,\text{m}$:
```text
Target Position: (1.20, 0.60) m
Configuration ELBOW_DOWN:
  th1: -9.82 deg (-0.1714 rad), th2: 84.26 deg (1.4706 rad)
  FK Re-verification Error: 0.00e+00 m

Configuration ELBOW_UP:
  th1: 62.96 deg (1.0988 rad), th2: -84.26 deg (-1.4706 rad)
  FK Re-verification Error: 0.00e+00 m
```

### Task 2: Implement Batch Reachability Evaluation
Create a test script evaluating multiple Cartesian targets across different quadrants and outside the workspace:
```python
import numpy as np
from inverse_kinematics_2d import ik_2link_planar

l1, l2 = 1.0, 0.8
test_targets = [
    (1.2, 0.6),    # Reachable in Quadrant I
    (-0.8, 0.9),   # Reachable in Quadrant II
    (0.5, -0.6),   # Reachable in Quadrant IV
    (2.5, 1.0),    # Unreachable (exceeds outer radius 1.8m)
    (0.05, 0.05)   # Unreachable (inside inner deadzone 0.2m)
]

for xt, yt in test_targets:
    res = ik_2link_planar(l1, l2, xt, yt)
    print(f"Target ({xt:.2f}, {yt:.2f}): Reachable? {res['elbow_down'] is not None}")
```

### Task 3: Automated Unit Testing
Run the automated pytest suite:
```bash
pytest 04-python-robotics/tests/test_ik.py -k "test_2link" -v
```

#### Expected Output:
```text
test_ik.py::test_2link_ik_consistency PASSED
test_ik.py::test_2link_ik_unreachable PASSED
```

---

## 4. Verification Checkpoints
- [ ] Both solutions (elbow-up and elbow-down) reproduce the commanded target with position error $< 10^{-6}\,\text{m}$.
- [ ] Targets outside the outer radius ($r > 1.8\,\text{m}$) return `None` gracefully without `NaN` or unhandled exceptions.
- [ ] Targets inside the inner radius ($r < 0.2\,\text{m}$) return `None` gracefully.

---

## 5. Extension Task: Continuous Cartesian Trajectory Tracking
Generate a straight line trajectory connecting point $\mathbf{A} = [0.8, 0.4]$ to point $\mathbf{B} = [1.2, 0.8]$ using 50 waypoints:
- Solve inverse kinematics for each waypoint along the line.
- Plot the resulting joint angles $\theta_1(t)$ and $\theta_2(t)$ over time.
- Discuss: What happens to joint continuity if your controller accidentally flips from the elbow-down branch to the elbow-up branch midway through the trajectory?

---

## 6. Lab Deliverables
Submit:
1. Your completed batch target evaluation script and terminal log.
2. Matplotlib visualization displaying both elbow-up and elbow-down configurations reaching target $(1.20, 0.60)$.
3. Joint angle trajectory plots from the extension task.
