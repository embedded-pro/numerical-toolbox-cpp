# Kinematics

Geometry and motion algorithms for serial kinematic chains.

## Algorithms

| Algorithm                                  | Description                                                                         |
|--------------------------------------------|-------------------------------------------------------------------------------------|
| [Forward Kinematics](ForwardKinematics.md) | Computes 3D joint positions from joint angles for serial chains — $O(n)$ complexity |
| [Inverse Kinematics](InverseKinematics.md) | Solves joint angles from a target end-effector position via Damped Least Squares — $O(k \cdot n)$ iterative |
