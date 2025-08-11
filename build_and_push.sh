#!/bin/bash
set -e  # Exit if any command fails

IMAGE_NAME="mahadev1632/navstate"

# Make sure we are in Jenkins workspace
cd "$WORKSPACE" || { echo "Workspace not found"; exit 1; }

# Authenticate to Docker Hub once
echo "$DOCKER_HUB_PASSWORD" | docker login -u "$DOCKER_HUB_USERNAME" --password-stdin

# Fetch all tags from remote
git fetch --tags

# Loop through tags that match pattern v1.0, v1.1, etc.
for TAG in $(git tag -l 'v[0-9]*.[0-9]*'); do
    echo "=== Processing tag: $TAG ==="

    # Checkout tag
    git checkout "$TAG"

    # Build Docker image for this tag
    docker build -t "$IMAGE_NAME:$TAG" .

    # Push image to Docker Hub
    docker push "$IMAGE_NAME:$TAG"

    echo "=== Finished pushing $IMAGE_NAME:$TAG ==="
done

# Go back to main branch (optional)
git checkout main
