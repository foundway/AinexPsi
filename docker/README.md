# Docker cheatsheet for AinexΨ

Docker image based on ROS2 Humble Perception plus additional Python modules

Build a Docker image:

``` % docker compose build``` OR ```% docker compose build --no-cache ```

Bring up the container:

```% docker compose up -d```

Enter the shell in the container:

```% docker exec -it ros-humble-container bash```

Put down the container:

```% docker compose down```
