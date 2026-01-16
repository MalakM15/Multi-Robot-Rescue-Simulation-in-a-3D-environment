#ifndef GRID_H
#define GRID_H

#include "config.h"

typedef struct {
    int obstacle;      // 1 = debris, 0 = free
    float heat;        // simulated heat sensor
    float co2;         // simulated CO2 sensor
    int survivor;      // 1 = survivor detected
    int risk;          // 0–3 risk level
} Cell;

// Global pointer to 3D grid (allocated at runtime)
extern Cell ***building;

// Grid allocation and cleanup
int allocate_grid(const Config *cfg);
void free_grid(const Config *cfg);

// Grid generation
void generate_obstacles(const Config *cfg);
void assign_risk_from_obstacles(const Config *cfg);
void simulate_sensors(const Config *cfg);
void detect_survivors(const Config *cfg);

// Visualization
int count_survivors(const Config *cfg);

#endif
