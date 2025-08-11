#!/bin/bash
set -e  # Exit immediately if a command fails
set -o pipefail

# ==== CONFIGURATION ====
GITHUB_REPO="https://github.com/Mahadev-cmd/NavState.git"
IMAGE_NAME="mahadev1632/navstate"

# ==== CHECK FOR INPUT TAG ====
if [[ -z "$1" ]]; then
    echo "❌ ERROR: No tag specified. Please provide a tag."
    exit 1
fi

TAG="$1"
echo "🔖 Using provided tag: $TAG"

# ==== DOCKER LOGIN ====
if [[ -z "$DOCKER_HUB_USERNAME" || -z "$DOCKER_HUB_PASSWORD" ]]; then
    echo "❌ ERROR: Docker Hub credentials not set in environment variables."
    exit 1
fi
echo "🔐 Logging in to Docker Hub..."
echo "$DOCKER_HUB_PASSWORD" | docker login -u "$DOCKER_HUB_USERNAME" --password-stdin

# ==== FETCH LATEST TAG FROM GIT ====
echo "📥 Fetching tags from remote..."
git fetch --tags --force

# ==== VERIFY TAG EXISTS ====
if ! git rev-parse "$TAG" >/dev/null 2>&1; then
    echo "❌ ERROR: Tag $TAG does not exist in the repository."
    exit 1
fi

# ==== PROCESS THE SPECIFIED TAG ====
echo "========================================"
echo "🏷  Processing tag: $TAG"
echo "========================================"

# Ensure clean working directory before checkout
git reset --hard
git clean -fd

# Checkout the tag
git checkout "$TAG"

# Build Docker image
echo "🐳 Building Docker image $IMAGE_NAME:$TAG..."
docker build -t "$IMAGE_NAME:$TAG" .

# Push Docker image
echo "📤 Pushing Docker image to Docker Hub..."
docker push "$IMAGE_NAME:$TAG"

echo "✅ Finished processing tag: $TAG"
echo

# Return to main branch for safety
git checkout main
echo "🎉 Tag $TAG built and pushed successfully."
