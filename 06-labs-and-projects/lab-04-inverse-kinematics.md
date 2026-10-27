# Lab 04: Planar Inverse Kinematics & Workspace Reachability

## 1. Laboratory Objectives
In this laboratory, you will:
1. Implement the analytical Law of Cosines inverse kinematics algorithm for a 2-link planar arm.
2. Program logic that computes both **Elbow-Up** and **Elbow-Down** solutions.
3. Detect unreachable targets (both outer radius $r > l_1 + l_2$ and inner deadzone $r < |l_1 - l_2|$).
4. Verify computed joint angles by piping them back through forward kinematics to confirm position errors $< 10^{-6}\,\text{m}$.

---

## 2. Mathematical Algorithm
Given link lengths $l_1, l_2$ and target $(x, y)$:
1. Distance: $r = \sqrt{x^2 + y^2}$.
2. If $r > l_1 + l_2$ or $r < |l_1 - l_2| \to$ Return `None`.
3. $\cos\theta_2 = \frac{x^2 + y^2 - l_1^2 - l_2^2}{2 l_1 l_2}$.
4. $\theta_2 = \pm \arccos(\cos\theta_2)$.
5. $\theta_1 = \text{atan2}(y, x) - \text{atan2}(l_2 \sin\theta_2, \; l_1 + l_2 \cos\theta_2)$.

---

## 3. Laboratory Tasks

### Task 1: Execute and Test
Run the pre-built planar IK engine:
```bash
python 04-python-robotics/inverse_kinematics_2d.py
```

### Task 2: Implement Batch Target Evaluation
Write a script that reads 5 target points from a list and computes solutions:
```python
targets = [
    (1.2, 0.5),   # Reachable
    (-0.8, 0.9),  # Reachable in Quadrant II
    (0.3, -0.4),  # Reachable in Quadrant IV
    (2.5, 1.0),   # Unreachable (too far)
    (0.05, 0.05)  # Unreachable (inside hole)
]
```

### Task 3: Unit Testing
Run the automated pytest test:
```bash
pytest -v 04-python-robotics/tests/test_ik.py -k "test_2link"
```

---

## 4. Verification Checkpoints
- Both solutions (elbow-up and elbow-down) reproduce the commanded Cartesian target within $10^{-6}\,\text{m}$.
- Out-of-bounds targets return `None` gracefully without throwing floating-point exceptions.

---

## 5. Lab Deliverables
1. Completed Python script with batch reachability testing.
2. Matplotlib visualization showing the 2-link arm in both elbow-up and elbow-down configurations reaching the same target.
