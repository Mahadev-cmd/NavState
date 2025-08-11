#!/bin/bash
set -e  # Exit immediately if a command fails
set -o pipefail

# Usage: ./build_and_push.sh <version-tag>

if [ "$#" -lt 1 ]; then
  echo "Usage: $0 <version-tag>"
  exit 1
fi

VERSION=$1

# ==== CONFIGURATION ====
GITHUB_REPO="https://github.com/Mahadev-cmd/NavState.git"
IMAGE_NAME="mahadev1632/navstate"

# ==== DOCKER LOGIN ====
if [[ -z "$DOCKER_HUB_USERNAME" || -z "$DOCKER_HUB_PASSWORD" ]]; then
    echo "❌ ERROR: Docker Hub credentials not set in environment variables."
    exit 1
fi
echo "🔐 Logging in to Docker Hub..."
echo "$DOCKER_HUB_PASSWORD" | docker login -u "$DOCKER_HUB_USERNAME" --password-stdin

# ==== FETCH LATEST TAGS ====
echo "📥 Fetching tags from remote..."
git fetch --tags --force

# ==== VERIFY TAG EXISTS ====
if ! git rev-parse "$VERSION" >/dev/null 2>&1; then
    echo "❌ ERROR: Tag $VERSION does not exist in the repository."
    exit 1
fi

# ==== PROCESS THE SPECIFIED TAG ====
echo "========================================"
echo "🏷  Processing tag: $VERSION"
echo "========================================"

# Ensure clean working directory before checkout
git reset --hard
git clean -fd

# Checkout the tag
git checkout "$VERSION"

# Build Docker image
echo "🐳 Building Docker image $IMAGE_NAME:$VERSION..."
docker build -t "$IMAGE_NAME:$VERSION" .

# Push Docker image
echo "📤 Pushing Docker image to Docker Hub..."
docker push "$IMAGE_NAME:$VERSION"

echo "✅ Finished processing tag: $VERSION"
echo

# Return to main branch for safety
git checkout main
echo "🎉 Tag $VERSION built and pushed successfully."
