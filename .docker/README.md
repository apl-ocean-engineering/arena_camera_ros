
# `arena_camera_ros` Docker readme

The [`Dockerfile`](Dockerfile) builds a self-contained image which includes the LucidVision Arena SDK and a catkin workspace which includes this software built from a clean git checkout.  The default command in the docker is:

> roslaunch arena_camera arena_camera_nodelet.launch


# Local development

The `docker-compose.yml` included in this directory is designed for local development on this repository.

To build the full Docker image:

```
docker compose build arena_camera
```

To start an instance **with the local repository bind-mounted as ~/ros_ws/src/arena_camera_ros**

```
docker compose run build_local
```

This allows for local development.
