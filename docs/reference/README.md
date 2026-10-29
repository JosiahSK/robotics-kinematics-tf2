# Robotics Reference Centre

Welcome to the **Robotics Reference Centre**. This section serves as a fast, searchable lookup hub for mathematical formulas, Denavit–Hartenberg parameters, ROS 2 TF2 commands, robotics terminology, and debugging guides.

Instead of re-reading multi-page chapters, use these cheat sheets for quick reference during lab sessions, programming assignments, and exam preparation.

---

## Reference Guides

| Reference Guide | Key Topics | Direct Link |
| :--- | :--- | :--- |
| **Robotics Glossary** | Definitions of 30+ core terms: $SO(3)$, $SE(3)$, Pieper criterion, DLS, Yoshikawa manipulability, Hamilton quaternions, TF2, Transient Local QoS | [**Open Glossary**](glossary.md) |
| **Kinematics Formula Sheet** | Forward kinematics, Law of Cosines IK, wrist center decoupling equations, geometric Jacobian, DLS SVD update, Lie algebra $SO(3)$ log-map | [**Open Formula Sheet**](kinematics-formula-sheet.md) |
| **DH Parameters Cheat Sheet** | Standard DH 4-parameter rules, common normal, frame assignment algorithm, EduArm-6DOF table, and DH-to-URDF conversion formulas | [**Open DH Cheat Sheet**](dh-parameters-cheat-sheet.md) |
| **Notation & Coordinate Frames** | Coordinate frame superscripts/subscripts (${}^A\mathbf{T}_B$, ${}^A\mathbf{p}$), right-hand rule, REP 103 standards, and SI unit conventions | [**Open Notation Guide**](notation-and-units.md) |
| **ROS 2 TF2 Command Cheat Sheet** | Essential CLI debugging commands (`tf2_echo`, `view_frames`, `tf2_monitor`), Python broadcaster/listener templates, and QoS setup | [**Open TF2 Cheat Sheet**](ros2-tf2-cheat-sheet.md) |
| **Troubleshooting & Common Pitfalls** | Diagnosing `atan2` argument order, degree/radian bugs, elbow branch flips, vanishing skew near $180^\circ$, and TF extrapolation errors | [**Open Troubleshooting FAQ**](troubleshooting-faq.md) |
| **Student Progress Checklist** | Self-tracking milestone checklist covering all theoretical modules, Python implementations, tests, and practical labs | [**Open Progress Checklist**](student-checklist.md) |

---

## How to Use This Reference Centre

1. **During Labs:** Keep the [TF2 Cheat Sheet](ros2-tf2-cheat-sheet.md) and [Troubleshooting FAQ](troubleshooting-faq.md) open while writing ROS 2 launch files and debugging frames in RViz.
2. **During Coding:** Refer to the [DH Cheat Sheet](dh-parameters-cheat-sheet.md) and [Kinematics Formula Sheet](kinematics-formula-sheet.md) when writing custom matrix transformations or Jacobian controllers.
3. **During Exam Prep:** Review the [Glossary](glossary.md) and verify your mastery with the [Student Progress Checklist](student-checklist.md).
