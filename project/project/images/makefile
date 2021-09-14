
INPUTS=$(wildcard *.mkv)
TARGETS=$(INPUTS:%.mkv=%.gif)

all: $(TARGETS)

$(TARGETS): %.gif: %.mkv
	ffmpeg -i $< -s 1080x658 $@

inspect:
	echo $(INPUTS)
	echo $(TARGETS)
