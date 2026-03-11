from setuptools import find_packages, setup

setup(
    name="lerobot_policy_act_augmented",
    version="0.1.0",
    description="ACT with configurable visual input transforms (LeRobot plugin)",
    packages=find_packages(),
    python_requires=">=3.10",
    install_requires=["lerobot", "lerobot_visual_transforms"],
)
