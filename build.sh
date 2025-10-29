docker buildx build \
  --platform linux/arm64 \
  --pull \
  -f .devcontainer/Dockerfile \
  -t aatmaj9/auv:1.0 \
  --push \
  .