# Adversarial falsification harness — drives the REAL firmware navigation.h.
# Header-only model + the firmware nav header; no Arduino/hardware dependency.
CXX = g++
FW_NAV_DIR = ../main/src/navigation
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra -I. -I$(FW_NAV_DIR)
TARGET = adversarial

all: $(TARGET)

$(TARGET): adversarial_main.cpp boat_model.hpp metrics.hpp $(FW_NAV_DIR)/navigation.h
	$(CXX) $(CXXFLAGS) -o $(TARGET) adversarial_main.cpp -lm
	@echo "✓ Built ./$(TARGET)"

# CI gate: baseline control only (must converge, else harness/nav suspect)
baseline: $(TARGET)
	./$(TARGET) --baseline

# Full campaign: baseline + all sensitivity sweeps + CSV
sweep: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET) adversarial.csv

.PHONY: all baseline sweep clean
