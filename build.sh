docker buildx build \
  --platform linux/arm64 \
  --pull \
  -f .devcontainer/Dockerfile_esp32 \
  -t aatmaj9/auv_with_esp32:1.0 \
  --push \
  .