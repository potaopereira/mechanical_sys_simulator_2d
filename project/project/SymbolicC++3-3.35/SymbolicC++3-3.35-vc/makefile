COMPILER=g++ -std=c++11
SOURCE_DIR=src/
OBJ_DIR=obj/
INCLUDE_DIR=include/
# where to look for header files 
INCLUDE_DIRS=-I include/ -I include/symbolic/
CPP_FILES=$(wildcard $(SOURCE_DIR)*.cpp)
OBJECT_FILES=$(CPP_FILES:$(SOURCE_DIR)%.cpp=$(OBJ_DIR)%.o)

# add dependencies (necessary header files)
$(OBJ_DIR)%.o: $(SOURCE_DIR)%.cpp
	$(COMPILER) -c $< -o $@ $(INCLUDE_DIRS)

# not a target file, include in .PHONY
all: $(OBJECT_FILES)

clean:
	rm -rf obj/*.o

.PHONY: all clean