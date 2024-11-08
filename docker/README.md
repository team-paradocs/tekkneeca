# Docker

> **Note:** Run all scripts from the root directory of the repository.

## Build the Docker image

```bash
./docker/tekkneeka.sh build
```

## Run the Docker container

```bash
./docker/tekkneeka.sh run
```
If the entrypoint is enabled, it will build the ROS workspace and source the new setup file on container start.

### Variants

- `./docker/tekkneeka.sh run sim` - Run sim version (no devices)
- `./docker/tekkneeka.sh run camera` - Run camera version (only video devices)
- `./docker/tekkneeka.sh run` - Run default version (all devices)

## Enter the Docker container

```bash
./docker/shell.sh
```
