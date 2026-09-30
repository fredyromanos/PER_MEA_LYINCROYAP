# Makefile pour la simulation AutoBoat
# Compile la simulation sans dépendances Arduino

CXX = g++
# SINGLE SOURCE OF TRUTH for navigation.h: compile against the FIRMWARE copy so
# the simulator always validates the exact logic the boat runs. The old local
# Simulation/navigation.h had drifted (stale pre-2026-06-10 algorithm) and the
# legacy -I../boat pointed at a nonexistent dir. Firmware nav dir is first on
# the path; Simulation/navigation.h is now a redirect to the same file.
FW_NAV_DIR = ../main/src/navigation
# Firmware src root: the sim also runs the REAL HeadingGate (GPS-only heading logic).
# mocks/ on the include path satisfies <Arduino.h> in the firmware headers it pulls in.
FW_SRC = ../main/src
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -g -I. -I$(FW_NAV_DIR) -I.. -I$(FW_SRC) -Imocks
LDFLAGS = -lm

# Répertoires source
EXPORT_DIR = export
MOCKS_DIR = mocks

# Sources de la simulation
SOURCES = \
	sim_main.cpp \
	sim_environment.cpp \
	sim_boat.cpp \
	$(EXPORT_DIR)/sim_html_export.cpp \
	$(MOCKS_DIR)/Arduino.cpp \
	$(MOCKS_DIR)/LoRa.cpp

# Fichiers objet
OBJECTS = $(SOURCES:.cpp=.o)
NAV_HEADER = $(FW_NAV_DIR)/navigation.h

# Firmware sources compiled into LOCAL objects (never write .o into the firmware tree)
FW_HEADING_GATE = $(FW_SRC)/control/HeadingGate.cpp
FW_OBJECTS = fw_heading_gate.o

# Cible exécutable
TARGET = boat_simulator

# Cibles principales
all: $(TARGET)

$(TARGET): $(OBJECTS) $(FW_OBJECTS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJECTS) $(FW_OBJECTS) $(LDFLAGS)
	@echo "✓ Build successful: ./$(TARGET)"

# Every object depends on the shared headers: a class layout change (e.g. new members in
# sim_boat.hpp) must rebuild ALL objects, or stale ones corrupt memory at run time.
SIM_HEADERS = sim_boat.hpp sim_environment.hpp mocks/Arduino.h mocks/LoRa.h mocks/SPI.h \
	$(EXPORT_DIR)/sim_html_export.hpp \
	$(FW_SRC)/control/HeadingGate.h $(FW_SRC)/core/Types.h $(FW_SRC)/config/Calibration.h

%.o: %.cpp $(NAV_HEADER) $(SIM_HEADERS)
	$(CXX) $(CXXFLAGS) -c $< -o $@

fw_heading_gate.o: $(FW_HEADING_GATE) $(FW_SRC)/control/HeadingGate.h $(FW_SRC)/config/Calibration.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJECTS) $(FW_OBJECTS) $(TARGET)
	rm -f output/*.csv output/*.html
	@echo "✓ Clean complete"

run: $(TARGET)
	./$(TARGET) 1

run-all: $(TARGET)
	@for i in 1 2 3 4 5 6; do \
		echo "\n=== Running scenario $$i ==="; \
		./$(TARGET) $$i; \
	done

# Run with the REAL forecast wind from Windy (needs network + key in IHM/.env).
# make run-windy [LAT=48.34] [LON=-4.52] [SCENARIO=1]
LAT ?= 48.34
LON ?= -4.52
SCENARIO ?= 1
run-windy: $(TARGET)
	@exports="$$(python3 fetch_wind.py $(LAT) $(LON))" || { echo "run-windy: could not fetch the Windy forecast, not running"; exit 1; }; \
	eval "$$exports"; ./$(TARGET) $(SCENARIO)

# Phony targets
.PHONY: all clean run run-all run-windy help

help:
	@echo "AutoBoat Simulator"
	@echo ""
	@echo "  make          - Build"
	@echo "  make run      - Build + run scenario 1"
	@echo "  make run-all  - Run all scenarios (1-6)"
	@echo "  make run-windy [LAT= LON= SCENARIO=] - Run with real Windy forecast wind"
	@echo "  make clean    - Remove build artifacts"
	@echo ""
	@echo "  Env overrides: SIM_WIND_DIR, SIM_WIND_SPEED, SIM_LEEWAY_GAIN"
	@echo "  Current:       ./boat_simulator N SPEED_MS DIR_DEG"
