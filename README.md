# ROS2-Docker-Development

## Recommended Tools:

- [ ] Visual Studio Code
    - [ ] [Remote Development extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack)
- [ ] [Docker](https://github.com/MHSeals/Docker-Tutorial)

## Running the Development Environment
Run 
```bash
xhost +
```
to ensure this works.
After cloning the repository, open up the project. VSCode will prompt you to reopen in a container. Hit yes, and then the coding environment for this repository will spin up. Once that finishes, you can start developing for this repository!

Follow this [link](https://code.visualstudio.com/remote/advancedcontainers/sharing-git-credentials) to learn how to pass your git credentials to the container. You can also refer to this [Stack Overflow Post](https://stackoverflow.com/questions/74704065/how-to-pass-git-ssh-credentials-from-wsl-to-vscode-dev-container) for reference.

### For NVIDIA Developers

You can pass your NVIDIA GPUs by changing `.devcontainer/devcontainer.json`'s 3rd line:

```diff
- 	"dockerComposeFile": "../docker-compose.yml",
+   "dockerComposeFIle": "../docker-compose.nvidia.yml",
```

### For Non-VSCode Users

After cloning this repository, you can run `docker-compose up -d` to start up the coding environment. From there, you can `docker exec -it <container name> /bin/bash` (you can use `docker ps` to view active containers) to access the container.

### For Nvidia Non-VSCode Users
Instead of running `docker-compose up -d`, run `docker-compose -f docker-compose.nvidia.yml up -d`

### RealSense

- You'll have to run `rviz2` to open the `rviz` program.
The `/dev` directory should be mounted to the container by default, do you should not have to worry about mounting them yourself to the container when you run `sudo docker-compose up`

- `realsense-viewer` is also an executable you can [run](https://github.com/2b-t/realsense-ros2-docker#2-launching), but I you'd probably have to plug in the camera first before running this executable.


## Running a ROS2 package

In the container, run `colcon build` to build all the packages. Once that's finished, you can source the generated setup file (ex: `source install/setup.bash`), and then run the package (ex: `ros2 run buoy_detection ai`).

- To view running nodes and topics, run `rqt_graph`.

## CONTRIBUTING

The main branch of this repository is protected, so in order to contribute to this project, you'll need to make **[pull requests](https://youtu.be/jRLGobWwA3Y)** to the main branch.

⭐ Although the linked video tells you to fork the repository, you really just need to make a separate branch from the main branch, or from another branch you want to contribute to. ⭐

Make sure you have another person reviewing your code before you merge it to the main branch! We want to make sure that it's code that we *want* to have in the main branch 😊
