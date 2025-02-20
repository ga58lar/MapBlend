<div align="center">
  <h1>MapBlend</h1>
  <h2>Consistent Long-Term Point Cloud Mapping with Prior Maps</h2>

  <br>

  [![Docker](https://badgen.net/badge/icon/docker?icon=docker&label)](https://www.docker.com/)
  ![Linux](https://img.shields.io/badge/Linux-FCC624?logo=linux&logoColor=black)
  [![ROS2](https://img.shields.io/badge/ros2-gray.svg)](https://docs.ros.org/en/jazzy/index.html)
  ![License](https://img.shields.io/badge/license-Apache%202.0-blue)
  ![Version](https://img.shields.io/badge/version-0.0.0-blue)
  
  <br>
  <img src="doc/seq00.gif" alt="Demo GIF">
</div>

  
## PRELIMINARY version

This is a PRELIMINARY version

## Install

```bash
git clone --recurse-submodules https://github.com/ga58lar/MapBlend.git

./docker/build_docker.sh
```

## Run

```bash
./docker/run_docker.sh <config_path> <map_path> <scan_path> <output_path> <x> <y> <z> <qx> <qy> <qz> <qw>
```

For a detailed instruction the the core module [OpenLiDARMap](https://github.com/TUMFTM/OpenLiDARMap), please refer to the official GitHub repository.

## Missing

- Object tracking module for robust movable object removal
- Life-long mapping module