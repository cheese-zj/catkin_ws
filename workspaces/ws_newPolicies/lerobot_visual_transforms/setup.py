from setuptools import find_packages, setup

setup(
    name="lerobot_visual_transforms",
    version="0.1.0",
    description="Composable visual input transforms for LeRobot ACT-based policies",
    packages=find_packages(),
    python_requires=">=3.10",
    install_requires=["torch", "torchvision"],
)
