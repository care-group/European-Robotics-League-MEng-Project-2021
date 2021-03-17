# MEng Group Project - European Robotic League
## Initial Installation
Install [Docker](https://www.docker.com/)
Verify Docker installation by running
```
docker info
```
Clone this repository
```
git clone https://github.com/chulme/european_robotic_league.git
cd european_robotic_league
```
Download all of the images necessary for running the simulator, [curtosy of hsr-project](https://github.com/hsr-project/tmc_wrs_docker).
In Windows
```
get-images.sh
```
In Mac
```
sh get-images.sh
```
Start the simulator
```
docker-compose up
```
Access the container through the [online IDE](http://localhost:3001) or locally-installed Visual Studio Code and run
```
cd /workspace
sh catkin_make_with_dep.sh
source devel/setup.bash --extend
```
## Development
- The simulator's screen http://localhost:3000
- IDE http://localhost:3001

## Installation of Additional Software
To use additional software, we must first add automated instructions to `Dockerfile` so that everyone can easily update their image. I have created the `additional_software` folder so that we can have a singular installation folder for this.

## Common Issues
If you are faced with errors complaining about `\r ` characters, on your local console (eg. Windows) you need to disable Git from converting line-breaks to Windows-style and reclone the repository, you do not need to pull the images again.
```
git config --global core.autocrlf false
```
