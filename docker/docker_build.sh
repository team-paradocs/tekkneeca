BRANCH_NAME=$(git rev-parse --abbrev-ref HEAD)
sudo docker build -f docker/Dockerfile -t paradockerimage:${BRANCH_NAME} .