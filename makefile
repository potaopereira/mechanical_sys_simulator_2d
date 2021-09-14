IMAGE=rigid_bodies_2d_img
CONTAINER=rigid_bodies_2d_cnt
BUILD=build

all: start

start: .create $(BUILD)
	docker start $(CONTAINER)
	docker \
	exec \
	-it \
	$(CONTAINER) \
	/bin/zsh -c " \
	cd project/project/build/gui \
	&& \
	./gui \
	"

inspect: .create
	docker start $(CONTAINER)
	docker \
	exec \
	-it \
	$(CONTAINER) \
	/bin/zsh

$(BUILD):
	docker start $(CONTAINER)
	docker \
	exec \
	-it \
	$(CONTAINER) \
	/bin/zsh -c " \
	cd project/project \
	&& \
	mkdir -p $@ \
	&& \
	cd $@ \
	&& \
	cmake ../project \
	&& \
	make \
	"

.create: .build
	xhost +local:docker
	docker create \
	-it \
	--name $(CONTAINER) \
	-v "$(PWD)":/home/$(shell id -nu)/project \
	-v /tmp/.X11-unix/:/tmp/.X11-unix \
	$(IMAGE)
	touch $@

.build: dockerfile.df
	docker build \
	--tag $(IMAGE) \
	--build-arg user=$(shell id -nu) \
	--file $^ \
	.
	touch $@

.PHONY: run