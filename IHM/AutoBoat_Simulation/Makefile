# IHM simulators.
#   simu_boat : legacy random-drift demo (unchanged)
#   simu_gps  : digital twin with realistic GPS + REAL firmware HeadingGate/navigation/heartbeat
SIM_DIR = ../../../../Simulation/simulation
FW_SRC  = ../../main/src
FW_NAV  = $(FW_SRC)/navigation
CXX      = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -I$(SIM_DIR) -I$(SIM_DIR)/mocks -I$(FW_NAV) -I$(FW_SRC)
BUILD    = build

GPS_SRCS = simu_gps.cpp $(SIM_DIR)/sim_environment.cpp $(SIM_DIR)/sim_boat.cpp $(FW_SRC)/control/HeadingGate.cpp
GPS_HDRS = $(SIM_DIR)/sim_boat.hpp $(SIM_DIR)/sim_environment.hpp $(FW_SRC)/control/HeadingGate.h \
           $(FW_SRC)/comm/Heartbeat.h $(FW_SRC)/config/Calibration.h $(FW_SRC)/core/Types.h $(FW_NAV)/navigation.h

all: simu_gps predict_path

# Objects go to build/ (never into Simulation/ or the firmware tree).
$(BUILD)/%.o:
	@mkdir -p $(BUILD)

simu_gps: $(GPS_SRCS) $(GPS_HDRS)
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) $(GPS_SRCS) -o $@ -lm
	@echo "built ./simu_gps"

PRED_SRCS = predict_path.cpp $(SIM_DIR)/sim_environment.cpp $(SIM_DIR)/sim_boat.cpp $(FW_SRC)/control/HeadingGate.cpp

predict_path: $(PRED_SRCS) $(GPS_HDRS)
	$(CXX) $(CXXFLAGS) $(PRED_SRCS) -o $@ -lm
	@echo "built ./predict_path"

simu_boat: simu_boat.cpp
	$(CXX) -std=c++17 -O2 simu_boat.cpp -o $@

clean:
	rm -rf $(BUILD) simu_gps predict_path

.PHONY: all clean
