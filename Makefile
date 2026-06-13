CXX := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -pthread -Iinclude -O2 -march=native

OBJDIR := build

TARGET := SmartVehicle
TEST_TARGET := SmartVehicleTest

COMMON_SRCS := \
src/Alert.cpp \
src/AlertEvaluator.cpp \
src/AdaptiveAlertPrioritizer.cpp \
src/CanBus.cpp \
src/Dashboard.cpp \
src/DriverProfile.cpp \
src/EcuHealthMonitor.cpp \
src/Logger.cpp \
src/Performance.cpp \
src/PerformanceGraphStats.cpp \
src/Sensor.cpp \
src/OtaUpdateSimulator.cpp \
src/CrashSafeRecorder.cpp \
src/ServiceOrientedComm.cpp \
src/DtcSimulator.cpp \
src/Statistics.cpp

COMMON_OBJS := $(patsubst src/%.cpp,$(OBJDIR)/%.o,$(COMMON_SRCS))

MAIN_OBJ := $(OBJDIR)/main.o
TEST_OBJ := $(OBJDIR)/testMain.o

.PHONY: all test clean run run-test

all: $(OBJDIR) logs $(TARGET)

test: $(OBJDIR) logs $(TEST_TARGET)

$(OBJDIR):
	mkdir -p $(OBJDIR)

logs:
	mkdir -p logs

$(TARGET): $(COMMON_OBJS) $(MAIN_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(TEST_TARGET): $(COMMON_OBJS) $(TEST_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(OBJDIR)/%.o: src/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

run: all
	./$(TARGET)

run-test: test
	./$(TEST_TARGET)

clean:
	rm -rf $(OBJDIR)
	rm -f $(TARGET)
	rm -f $(TEST_TARGET)