IMAGE=build_2d_rigid_body_image
BUILD_TARGET=.build

run = \
docker run \
--rm \
-it \
-v $(PWD)/../:/home/$(shell id -nu) \
-w /home/$(shell id -nu) \
$(IMAGE) \
$(1)

all: install linuxdeploy-x86_64.AppImage
	export LD_LIBRARY_PATH="$${LD_LIBRARY_PATH}:build/AppDir/usr/lib" \
	&& \
	./linuxdeploy-x86_64.AppImage --appdir build/AppDir --output appimage
	mkdir gui-x86_64.AppImage.home

extra:
	cp build/AppDir/usr/bin/config.yaml gui-x86_64.AppImage.home/


check_application_runs:
	export LD_LIBRARY_PATH="$${LD_LIBRARY_PATH}:build/AppDir/usr/lib" && ./build/AppDir/usr/bin/gui

linuxdeploy-x86_64.AppImage:
	wget https://github.com/linuxdeploy/linuxdeploy/releases/download/continuous/linuxdeploy-x86_64.AppImage
	chmod +x linuxdeploy-x86_64.AppImage

install: compile
	# now "install"
	$(call run, make --directory=appimage/build install DESTDIR=AppDir)

compile: build/makefile
	# build the application on 4 CPU cores
	$(call run, make --directory=appimage/build -j 4)

# directory CMAKE_INSTALL_PREFIX is prepended onto all install directories
# configure build system
# the flags below are the bare minimum that is needed
# the app might define additional variables that might have to be set
build/makefile: build $(IMAGE)
	$(call run, \
	cmake \
	-DCMAKE_INSTALL_PREFIX=/usr \
	-S project/project \
	-B appimage/build \
	)

build:
	mkdir build

# check image exists, otherwise build it
$(IMAGE):
	@# discard output of docker image inspect into /dev/null
	docker image inspect $@ > /dev/null || make $(BUILD_TARGET)

$(BUILD_TARGET): dockerfile.df
	docker build \
	--build-arg user=$(shell id -nu) \
	--tag $(IMAGE) \
	--file $^ \
	.
	@# phony target
	touch $@

inspect: dockerfile.df
	$(call run, /bin/bash)

clean:
	rm -rf build
	rm linuxdeploy-x86_64.AppImage