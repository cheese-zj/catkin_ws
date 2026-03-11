from setuptools import find_packages, setup

setup(
    name="lerobot_policy_act_temporal",
    version="0.1.0",
    description="ACT + Temporal Self-Attention via Recurrent Feature Buffer (LeRobot plugin)",
    packages=find_packages(),
    python_requires=">=3.10",
    install_requires=["lerobot"],
)
