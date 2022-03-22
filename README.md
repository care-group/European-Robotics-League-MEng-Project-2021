# European Robotics League - Consumer

Heriot-Watt University MEng Group Project (2022)

## Setting Up Your Workspace

### Requirements

- [Docker](https://www.docker.com/)

### Steps

To obtain all of the docker images necessary to build and running the simulator container, [courtesy of hsr-project](https://github.com/hsr-project/tmc_wrs_docker), run this once after cloning:
```bash
./get-images.sh
```

To start the simulator container, run this each time:
```bash
docker-compose up
```

Access the container through the [online IDE](http://localhost:3001) or locally in another terminal, and run the following setup:
```bash
cd /workspace
chmod +x catkin_make_with_dep.sh
./catkin_make_with_dep.sh
source devel/setup.bash --extend
```

## Development

- The simulator's screen http://localhost:3000
- IDE http://localhost:3001

### Installation of Additional Software

To use additional software, the installation must be scripted to run in the `Dockerfile` so that everyone can easily update their image.

### Common Issues

To access the docker container directly in a terminal window of your choosing, run:
```bash
command docker exec -it european-robotic-league_workspace_1 /bin/bash
```

If you are faced with errors complaining about `\r ` characters, on your local console (eg. Windows) you need to disable Git from converting line-breaks to Windows-style and reclone the repository, you do not need to pull the images again.
``` bash
git config --global core.autocrlf false
```
