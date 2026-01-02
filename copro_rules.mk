COPRO_CXX=g++
COPRO_CXXFLAGS += -Wall -Wextra -std=c++17
LIB_NAME=reveillib
SHARED_LIB_FILE=$(LIB_NAME).so

all: $(SHARED_LIB_FILE)

$(SHARED_LIB_FILE) : 

.PHONY: clean
clean:
  