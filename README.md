# ROS2-Docker-Development

## Recommended Tools:

- [ ] Visual Studio Code
    - [ ] [Remote Development extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack)
- [ ] [Docker](https://github.com/MHSeals/Docker-Tutorial)

## Running the Development Environment

After cloning the repository, open up the project. VSCode will prompt you to reopen in a container. Hit yes, and then the coding environment for this repository will spin up. Once that finishes, you can start developing for this repository!

### For NeoVim Developers

After cloning this repository, you can run `sudo docker-compose up` to start up the coding environment. From there, you can `docker exec -it ros2-docker-development /bin/bash` to access the container.

## CONTRIBUTING

The main branch of this repository is protected, so in order to contribute to this project, you'll need to make **[pull requests](https://youtu.be/jRLGobWwA3Y)** to the main branch.

⭐ Although the linked video tells you to fork the repository, you really just need to make a separate branch from the main branch, or from another branch you want to contribute to. ⭐

Make sure you have another person reviewing your code before you merge it to the main branch! We want to make sure that it's code that we *want* to have in the main branch 😊