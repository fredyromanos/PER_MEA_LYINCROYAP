#include <iostream>
#include <fstream>
#include <cstdio>
#include <unistd.h> 
#include <chrono>
#include <random>

int main() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> drift(-0.0001, 0.0001);
    
    // Variables de simulation
    double lat=48.360687;
    double lon = -4.565710;
    int battery = 98;

    std::ofstream serialPort("/tmp/ttySimu");
    if (!serialPort.is_open()) return 1;

    std::cout << "Simulation synchronisée avec l'IHM... Envoi vers /tmp/ttySimu" << std::endl;

    while(true) {
        lat += drift(gen);
        lon += drift(gen);
        if (battery > 5) battery--;

        char message[1024];
        // On aligne les clés JSON sur le script JS (ex: motor_power au lieu de motor)
        snprintf(message, sizeof(message),
            "{"
                "\"timestamp\": %ld,"
                "\"message\": {"
                    "\"mode\": \"AUTONOMOUS\","
                    "\"location\": [%.10f, %.10f],"
                    "\"servos\": {\"sail\": 45, \"rudder\": 90},"
                    "\"heading\": 185,"
                    "\"wind\": 120,"
                    "\"waypoints\": {\"total\": 5, \"current\": 2},"
                    "\"battery\": %d,"
                    "\"motor_power\": {\"power\": 12.5, \"percent\": 45.0}"
                "}"
            "}",
            (long)std::time(nullptr), // Ajout du timestamp attendu par ton JS
            lat, lon, 
            battery
        );

        serialPort << message << std::endl;
        serialPort.flush(); 

        usleep(1000000); 
    }
    return 0;
}