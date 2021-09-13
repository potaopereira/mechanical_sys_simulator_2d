IPEFILE=drawing

# build image using latex.df dockerfile
build:
	docker build \
	--tag ipe \
	--build-arg user=$(shell id -nu) \
	--build-arg group=$(shell id -ng) \
	--file ipe.df \
	.

echo_ids:
	@echo "user id"	
	@echo $(shell id -nu)
	@echo "group id"
	@echo $(shell id -ng)

run:
	docker run --rm -it \
	-e DISPLAY=$(DISPLAY) \
	-v /tmp/.X11-unix/:/tmp/.X11-unix \
	-v $(PWD):/home/wd \
	--privileged \
	ipe \
	ipe $(IPEFILE).ipe

print:
	docker run --rm -it \
	-e DISPLAY=$(DISPLAY) \
	-v /tmp/.X11-unix/:/tmp/.X11-unix \
	-v $(PWD):/home/wd \
	ipe \
	ipetoipe -pdf $(IPEFILE).ipe $(IPEFILE).pdf

inspect:
	docker run --rm -it \
	ipe \
	/bin/bash

.phony: build
