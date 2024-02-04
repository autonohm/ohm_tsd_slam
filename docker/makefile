image_version=humble

build-multiarch-and-push:
	docker buildx build --platform linux/amd64,linux/arm64/v8 -t franc0r/slam:$(image_version) --push .
