docker buildx build \
  --platform linux/arm64 \
  --pull \
  -f .devcontainer/Dockerfile \
  -t aatmaj9/sookshma-jetson:9.0 \
  --push \
  .