docker buildx build \
  --platform linux/arm64 \
  --pull \
  -f .devcontainer/Dockerfile_sauvc \
  -t aatmaj9/sauvc:1.0 \
  --push \
  .