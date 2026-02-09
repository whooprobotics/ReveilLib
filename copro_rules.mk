COPRO_CXX=aarch64-linux-gnu-g++
COPRO_CXXFLAGS += -Wall -Wextra -std=c++17 -DPLATFORM_RASPI
COPRO_INCLUDE_PATH=include/
COPRO_LIB_NAME=reveillib
SHARED_LIB_FILE=bin/$(COPRO_LIB_NAME)_copro.so

SRCS := $(shell find src -name "*.cc")
OBJS := $(patsubst src/%.cc, bin/copro/%.o, $(SRCS))

copro-lib: $(SHARED_LIB_FILE)

$(SHARED_LIB_FILE) : $(OBJS)
	$(COPRO_CXX) -shared $^ -o $@

bin/copro/%.o: src/%.cc
	@mkdir -p $(dir $@)
	$(COPRO_CXX) $(COPRO_CXXFLAGS) -I$(COPRO_INCLUDE_PATH) -fPIC -c $< -o $@

.PHONY: clean-copro

clean-copro:
	@echo Cleaning reveillib_copro
	-$Drm -rf bin/copro
	-$Drm -rf $(SHARED_LIB_FILE)
