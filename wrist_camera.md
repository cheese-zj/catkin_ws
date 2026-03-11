### Result files
- left: `/home/jameszhao2004/catkin_ws/src/zeno-wholebody-intervation/zeno-wholebody-robot/robot_side/cam_calibration/data/samples/left/handeye_axxb_20260310_224414.json`
- right: `/home/jameszhao2004/catkin_ws/src/zeno-wholebody-intervation/zeno-wholebody-robot/robot_side/cam_calibration/data/samples/right/handeye_axxb_20260310_230810.json`

### Pose results (gripper_base -> camera_optical_frame)
| side | parent -> child | samples | t (m) [x, y, z] | q (xyzw) | euler xyz (deg) |
|---|---|---:|---|---|---|
| left | `arm_left/gripper_base -> realsense_left_color_optical_frame` | 30 | `[-0.01020, -0.09698, 0.04891]` | `[-0.18729, 0.00624, -0.02959, 0.98184]` | `[-21.6009, 0.0669, -3.4653]` |
| right | `arm_right/gripper_base -> realsense_right_color_optical_frame` | 30 | `[-0.00460, -0.08867, 0.04095]` | `[-0.17516, -0.00219, -0.00136, 0.98454]` | `[-20.1763, -0.2744, -0.1095]` |

### Consistency metrics
| side | all_pass | position std xyz (m) | translation residual median (m) | orientation error median (deg) |
|---|---|---|---:|---:|
| left | `false` | `[0.00468, 0.00478, 0.00983]` | `0.00849` | `2.5050` |
| right | `true` | `[0.00490, 0.00771, 0.00758]` | `0.01094` | `1.2376` |

### Conclusion
- right 结果达标（`all_pass=true`），可直接用于 TF 发布。
- left 结果基本可用，但旋转中位误差略超阈值（2.505° > 2.0°），建议补采大姿态样本后再重算一版。
