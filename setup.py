from setuptools import setup, find_packages

setup(
    name="torqea",
    version="1.0.0",
    description="Python SDK for TORQEA J-series robotic joint actuators (CANopen / CiA 402)",
    long_description=open("README.md", encoding="utf-8").read(),
    long_description_content_type="text/markdown",
    author="TORQEA Robotics Co., Ltd.",
    url="https://github.com/torqea/torqea-joint-sdk",
    packages=find_packages(),
    install_requires=["python-can>=4.0"],
    python_requires=">=3.8",
    license="MIT",
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Topic :: Scientific/Engineering",
        "Intended Audience :: Developers",
    ],
)
