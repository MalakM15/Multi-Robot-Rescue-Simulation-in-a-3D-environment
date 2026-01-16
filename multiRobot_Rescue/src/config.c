#include "all_headers.h"

int load_config(const char *filename, Config *cfg) {
    // Set default values
    cfg->mutation_rate = 0.1;
    cfg->elitism_percent = 10;
    cfg->pool_size = 4;
    cfg->max_survivors_per_robot = 20;
    
    FILE *file = fopen(filename, "r");
    if (!file) {
        perror("Error opening config file");
        return -1;
    }

    char line[256];
    char key[64];
    char value[64];

    while (fgets(line, sizeof(line), file)) {
        size_t len = strlen(line);
        while (len > 0 && (line[len-1] == '\n' || line[len-1] == '\r' || line[len-1] == ' ')) {
            line[len-1] = '\0';
            len--;
        }
        
        if (line[0] == '#' || line[0] == '\0') {
            continue;
        }

        // Parse 
        if (sscanf(line, " %63[^=] = %63s", key, value) == 2) {
            len = strlen(key);
            while (len > 0 && key[len-1] == ' ') {
                key[len-1] = '\0';
                len--;
            }
            
            if (strcmp(key, "GRID_X") == 0) cfg->grid_x = atoi(value);
            else if (strcmp(key, "GRID_Y") == 0) cfg->grid_y = atoi(value);
            else if (strcmp(key, "GRID_Z") == 0) cfg->grid_z = atoi(value);
            else if (strcmp(key, "OBSTACLE_DENSITY") == 0) cfg->obstacle_density = atof(value);
            else if (strcmp(key, "HEAT_THRESHOLD") == 0) cfg->heat_threshold = atof(value);
            else if (strcmp(key, "CO2_THRESHOLD") == 0) cfg->co2_threshold = atof(value);
            else if (strcmp(key, "ROBOT_COUNT") == 0) cfg->robot_count = atoi(value);
            else if (strcmp(key, "POPULATION_SIZE") == 0) cfg->population_size = atoi(value);
            else if (strcmp(key, "GENERATIONS") == 0) cfg->generations = atoi(value);
            else if (strcmp(key, "MUTATION_RATE") == 0) cfg->mutation_rate = atof(value);
            else if (strcmp(key, "ELITISM_PERCENT") == 0) cfg->elitism_percent = atoi(value);
            else if (strcmp(key, "POOL_SIZE") == 0) cfg->pool_size = atoi(value);
            else if (strcmp(key, "MAX_SURVIVORS_PER_ROBOT") == 0) cfg->max_survivors_per_robot = atoi(value);
        }
    }

    if (cfg->pool_size <= 0) {
        cfg->pool_size = 1;  // Ensure at least one worker
    }
    
    // Validate max_survivors_per_robot
    if (cfg->max_survivors_per_robot <= 0) {
        cfg->max_survivors_per_robot = 20;  // Default if not set or invalid
    }
    

    fclose(file);
    return 0;
}
