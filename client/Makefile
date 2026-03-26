CXX      := g++

#CXXFLAGS := -pedantic-errors -Wall -Wextra -Werror -g --std=c++17
CXXFLAGS := -g --std=c++17
LDFLAGS  := -L./ManusSDK/lib -lManusSDK -lncurses -L/usr/local/lib -lprotobuf -lzmq -Wl,-rpath=./ManusSDK/lib:/usr/local/lib
BUILD    := ./
OBJ_DIR  := $(BUILD)/objects
APP_DIR  := $(BUILD)
TARGET   := SharpaManusClient.out
INCLUDE  := -I./ManusSDK/include -I/usr/include/eigen3
SRC      :=  \
   $(wildcard *.cpp) \
   $(wildcard *.cc)

OBJECTS  := $(patsubst %.cpp,$(OBJ_DIR)/%.o,$(filter %.cpp,$(SRC))) \
            $(patsubst %.cc,$(OBJ_DIR)/%.o,$(filter %.cc,$(SRC)))
DEPENDENCIES \
         := $(OBJECTS:.o=.d)

all: build $(APP_DIR)/$(TARGET)

$(OBJ_DIR)/%.o: %.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) $(INCLUDE) -c $< -MMD -o $@

$(OBJ_DIR)/%.o: %.cc
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) $(INCLUDE) -c $< -MMD -o $@

$(APP_DIR)/$(TARGET): $(OBJECTS)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -o $(APP_DIR)/$(TARGET) $^ $(LDFLAGS)
	
-include $(DEPENDENCIES)

.PHONY: all build clean debug release info

build:
	@mkdir -p $(APP_DIR)
	@mkdir -p $(OBJ_DIR)

debug: CXXFLAGS += -DDEBUG -g
debug: all

release: CXXFLAGS += -g
release: all

clean:
	-@rm $(TARGET)
	-@rm -rvf $(OBJ_DIR)/*

info:
	@echo "[*] Application dir: ${APP_DIR}     "
	@echo "[*] Object dir:      ${OBJ_DIR}     "
	@echo "[*] Sources:         ${SRC}         "
	@echo "[*] Objects:         ${OBJECTS}     "
	@echo "[*] Dependencies:    ${DEPENDENCIES}"
