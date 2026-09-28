# path_sim: MainTask::path_run() の経路生成をホスト(PC)で再現するコマンド。
# 経路生成はファームのソースをそのままビルドする(写しを持たない)。
# Param Console の迷路タブ(/api/maze/path)が実行前に `make -s` を呼ぶので、
# ファームのソースを変えれば次の計算から反映される。
#
#   make            build/path_sim を作る
#   build/path_sim < input.json > result.json   (ファームの printf は stderr)

ROOT := $(abspath ../..)
ARDUINOJSON := $(ROOT)/build/_deps/arduinojson-src/src
BUILD := build

ifeq ($(wildcard $(ARDUINOJSON)/ArduinoJson.h),)
$(error ArduinoJson がありません ($(ARDUINOJSON))。先にファームを cmake configure してください)
endif

CXX ?= g++
INCLUDES := -Istub -I$(ROOT) -I$(ROOT)/include -I$(ROOT)/include/search \
            -I$(ROOT)/include/action -I$(ARDUINOJSON)
CXXFLAGS := -std=gnu++20 -O2 -MMD -MP $(INCLUDES)

# ファームのソース(警告はファーム側のビルドで見るので、ここでは出さない)
FW_SRCS := $(ROOT)/src/search/logic.cpp \
           $(ROOT)/src/action/path_creator.cpp \
           $(ROOT)/src/action/trajectory_creator.cpp
FW_OBJS := $(patsubst $(ROOT)/src/%.cpp,$(BUILD)/fw/%.o,$(FW_SRCS))
HOST_OBJS := $(BUILD)/main.o

$(BUILD)/path_sim: $(FW_OBJS) $(HOST_OBJS)
	$(CXX) $^ -o $@

$(BUILD)/fw/%.o: $(ROOT)/src/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -w -c $< -o $@

$(BUILD)/main.o: main.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -Wall -Wno-attributes -c $< -o $@

clean:
	rm -rf $(BUILD)

.PHONY: clean
-include $(FW_OBJS:.o=.d) $(HOST_OBJS:.o=.d)
