#ifndef CONFIG_H
#define CONFIG_H

typedef struct {
    // Grid
    int grid_x, grid_y, grid_z;
    float obstacle_density;

    // Sensor thresholds
    float heat_threshold;
    float co2_threshold;

    // Robots and survivors
    int robot_count;

    // Genetic Algorithm
    int population_size;
    int generations;
    double mutation_rate;
    int elitism_percent;
    int pool_size;
    int max_survivors_per_robot;  // Maximum survivors each robot can rescue
} Config;

int load_config(const char *filename, Config *cfg);

#endif
