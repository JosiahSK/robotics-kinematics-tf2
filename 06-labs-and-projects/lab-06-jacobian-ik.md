# Lab 06: Jacobian Inverse Kinematics & Singularity Handling

## 1. Laboratory Objectives
In this laboratory, you will:
1. Implement a complete numerical inverse kinematics solver for a 6-DOF robot using the Geometric Jacobian.
2. Formulate orientation error using the Lie-algebra cross-product method in $SO(3)$.
3. Implement Damped Least Squares (Levenberg-Marquardt) to guarantee numerical stability near singularities.
4. Enforce physical joint hardstops using angle clamping.
5. Track condition number $\kappa(\mathbf{J})$ and Yoshikawa manipulability $w$ throughout the motion.

---

## 2. Mathematical Algorithm
At each iteration $k$:
1. Compute current pose $T(\mathbf{q}_k)$ via forward kinematics.
2. Compute position error: $\mathbf{e}_p = \mathbf{p}_d - \mathbf{p}_k$.
3. Compute orientation error: $\mathbf{e}_o = \frac{1}{2}(\mathbf{n}_k \times \mathbf{n}_d + \mathbf{s}_k \times \mathbf{s}_d + \mathbf{a}_k \times \mathbf{a}_d)$.
4. If $\|\mathbf{e}_p\| < \epsilon_p$ and $\|\mathbf{e}_o\| < \epsilon_o \to$ **Converged!**
5. Compute geometric Jacobian $\mathbf{J}(\mathbf{q}_k)$.
6. Compute DLS step: $\Delta \mathbf{q} = \mathbf{J}^T (\mathbf{J}\mathbf{J}^T + \lambda^2 \mathbf{I})^{-1} \begin{bmatrix} \mathbf{e}_p \\ \mathbf{e}_o \end{bmatrix}$.
7. Update angles: $\mathbf{q}_{k+1} = \text{clip}(\mathbf{q}_k + \Delta \mathbf{q}, \; \mathbf{q}_{\min}, \; \mathbf{q}_{\max})$.

---

## 3. Laboratory Tasks

### Task 1: Run the Numerical Solver
Execute the 6-DOF IK module:
```bash
python 04-python-robotics/inverse_kinematics_6dof.py
```
Observe the convergence behavior:
- How many iterations were needed? (Typically 8 to 25 iterations for reasonable seeds).
- What was the final position error? ($< 10^{-4}\,\text{m}$).
- What was the final orientation error? ($< 10^{-3}$).

### Task 2: Singularity Stress Test
Test the solver with a target that forces the robot into a wrist singularity ($\theta_5 \to 0$):
- Verify that standard pseudoinverse blows up while Damped Least Squares ($\lambda = 0.05$) remains well-bounded.

### Task 3: Automated Pytest Suite
Run the full automated test suite:
```bash
pytest -v 04-python-robotics/tests/test_ik.py
pytest -v 04-python-robotics/tests/test_jacobian.py
```

---

## 4. Verification Checkpoints
- The solver converges to within $0.1\,\text{mm}$ position error and $< 0.05^\circ$ orientation error.
- Unreachable targets gracefully report failure after `max_iterations` without throwing unhandled exceptions.
- All computed joint angles strictly lie within configured hardware limits.

---

## 5. Lab Deliverables
1. Convergence plot showing error norm $\|\mathbf{e}\|$ versus iteration index $k$.
2. Table comparing standard pseudo-inverse joint velocities against DLS joint velocities near a singularity.
