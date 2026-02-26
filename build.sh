docker buildx build \
  --platform linux/arm64 \
  --pull \
  --no-cache \
  -f .devcontainer/Dockerfile_portenta_zedless \
  -t aatmaj9/auv_portenta_zedless:1.0 \
  --push \
  .