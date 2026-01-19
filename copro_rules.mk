COPRO_CXX=g++
COPRO_CXXFLAGS += -Wall -Wextra -std=c++17 -DPLATFORM_COPRO
COPRO_INCLUDE_PATH=include/
LIB_NAME=reveillib
SHARED_LIB_FILE=bin/$(LIB_NAME)_copro.so

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
	rm -rf bin/copro
	rm $(SHARED_LIB_FILE)
