BRANCH_NAME=$(git rev-parse --abbrev-ref HEAD)
# Replace any non-alphanumeric characters with a hyphen, making it Docker tag friendly
MODIFIED_BRANCH_NAME=$(echo "$BRANCH_NAME" | sed 's/[^a-zA-Z0-9_\-]/-/g')
sudo docker build -f docker/Dockerfile -t paradockerimage:${MODIFIED_BRANCH_NAME} .
