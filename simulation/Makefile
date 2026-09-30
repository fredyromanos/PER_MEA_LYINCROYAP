# Makefile pour la simulation AutoBoat
# Compile la simulation sans dépendances Arduino

CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -g -MMD -MP -I. -I../main/src/navigation
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
	$(MOCKS_DIR)/LoRa.cpp \
	$(EXPORT_DIR)/sim_vv_archive.cpp

# Fichiers objet
OBJECTS = $(SOURCES:.cpp=.o)
DEPS = $(OBJECTS:.o=.d)
NAV_HEADER = ../main/src/navigation/navigation.h

# Cible exécutable
TARGET = boat_simulator

# Cibles principales
all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJECTS) $(LDFLAGS)
	@echo "✓ Build successful: ./$(TARGET)"

%.o: %.cpp $(NAV_HEADER)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJECTS) $(DEPS) $(TARGET)
	rm -f output/*.csv output/*.html
	@echo "✓ Clean complete"

run: $(TARGET)
	./$(TARGET) 1

run-all: $(TARGET)
	@for i in 1 2 3 4 5 6 7; do \
		echo "\n=== Running scenario $$i ==="; \
		./$(TARGET) $$i; \
	done

# Phony targets
.PHONY: all clean run run-all help

-include $(DEPS)

help:
	@echo "AutoBoat Simulator"
	@echo ""
	@echo "  make          - Build"
	@echo "  make run      - Build + run scenario 1"
	@echo "  make run-all  - Run all scenarios (1-7)"
	@echo "  make clean    - Remove build artifacts"
