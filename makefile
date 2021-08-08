IMAGE=rigid_bodies_2d_img

all: run

run: .build
	docker \
	run \
	--rm \
	-it \
	-v "$(PWD)"/project:/home/$(shell id -nu)/project \
	-w /home/$(shell id -nu)/project \
	$(IMAGE) \
	/bin/bash -c " \
	mkdir -p build && cd build && cmake ../project && make \
	"

inspect: .build
	docker \
	run \
	--rm \
	-it \
	-v "$(PWD)"/project:/home/$(shell id -nu)/project \
	-w /home/$(shell id -nu)/project \
	$(IMAGE) \
	/bin/bash

.build: dockerfile.df
	docker build \
	--tag $(IMAGE) \
	--build-arg user=$(shell id -nu) \
	--file $^ \
	.
	touch $@

.PHONY: run