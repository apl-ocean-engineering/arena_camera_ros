# `arena_camera_ros2` Docker readme

The [`Dockerfile`](Dockerfile) builds a self-contained image which includes the LucidVision Arena SDK and a colcon workspace which includes this software built from a clean git checkout.  The default command is:

> roslaunch arena_camera arena_camera_nodelet.launch


# Local development

The `docker-compose.yml` included in this directory is designed for local development on this repository.

To build the base Docker image:

```
docker compose build
```

To start an instance **with the local repository bind-mounted as ~/ros_ws/src/arena_camera_ros**

```
docker compose run arena_camera /bin/bash
```

This allows for local development.
