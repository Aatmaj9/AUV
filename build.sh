docker buildx build \
  --platform linux/arm64 \
  --pull \
  -f .devcontainer/Dockerfile_zedless \
  -t aatmaj9/auv_zedless:1.0 \
  --push \
  .