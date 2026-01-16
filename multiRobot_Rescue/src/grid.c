#include "all_headers.h"

Cell ***building = NULL;

int allocate_grid(const Config *cfg) {
    if (!cfg) return -1;
    
    // Allocate 3D grid: building[z][y][x]
    building = (Cell ***)malloc(cfg->grid_z * sizeof(Cell **));
    if (!building) return -1;
    
    for (int z = 0; z < cfg->grid_z; z++) {
        building[z] = (Cell **)malloc(cfg->grid_y * sizeof(Cell *));
        if (!building[z]) {
            // Cleanup on failure
            for (int zz = 0; zz < z; zz++) {
                for (int y = 0; y < cfg->grid_y; y++) {
                    free(building[zz][y]);
                }
                free(building[zz]);
            }
            free(building);
            building = NULL;
            return -1;
        }
        
        for (int y = 0; y < cfg->grid_y; y++) {
            building[z][y] = (Cell *)malloc(cfg->grid_x * sizeof(Cell));
            if (!building[z][y]) {
                // Cleanup on failure
                for (int zz = 0; zz < z; zz++) {
                    for (int yy = 0; yy < cfg->grid_y; yy++) {
                        free(building[zz][yy]);
                    }
                    free(building[zz]);
                }
                for (int yy = 0; yy < y; yy++) {
                    free(building[z][yy]);
                }
                free(building[z]);
                free(building);
                building = NULL;
                return -1;
            }
            
            // Initialize cells
            for (int x = 0; x < cfg->grid_x; x++) {
                building[z][y][x].obstacle = 0;
                building[z][y][x].heat = 0.0;
                building[z][y][x].co2 = 0.0;
                building[z][y][x].survivor = 0;
                building[z][y][x].risk = 0;
            }
        }
    }
    
    return 0;
}

void free_grid(const Config *cfg) {
    if (!building || !cfg) return;
    
    for (int z = 0; z < cfg->grid_z; z++) {
        if (building[z]) {
            for (int y = 0; y < cfg->grid_y; y++) {
                if (building[z][y]) {
                    free(building[z][y]);
                }
            }
            free(building[z]);
        }
    }
    free(building);
    building = NULL;
}

void generate_obstacles(const Config *cfg) {
    if (!building || !cfg) return;
    
    // obstacle_density is between 0.0 and 1.0
    srand(time(NULL));
    
    int total_cells = cfg->grid_x * cfg->grid_y * cfg->grid_z;
    
    // Special case: density = 1.0 
    if (cfg->obstacle_density >= 1.0) {
        for (int z = 0; z < cfg->grid_z; z++) {
            for (int y = 0; y < cfg->grid_y; y++) {
                for (int x = 0; x < cfg->grid_x; x++) {
                    building[z][y][x].obstacle = 1;
                }
            }
        }
        return;
    }
    
    int valid_cells = total_cells;
    if (valid_cells <= 0) {
        fprintf(stderr, "Error: Grid too small to place obstacles.\n");
        return;
    }
    
    typedef struct { int x, y, z; } CellPos;
    CellPos *cell_list = malloc(valid_cells * sizeof(CellPos));
    if (!cell_list) return;  
    
    // Build list of all cells
    int idx = 0;
    for (int z = 0; z < cfg->grid_z; z++) {
        for (int y = 0; y < cfg->grid_y; y++) {
            for (int x = 0; x < cfg->grid_x; x++) {
                cell_list[idx].x = x;
                cell_list[idx].y = y;
                cell_list[idx].z = z;
                idx++;
            }
        }
    }
    
    // Calculate obstacle count
    int obstacle_count = (int)(valid_cells * cfg->obstacle_density);
    if (obstacle_count > valid_cells) obstacle_count = valid_cells;
    
    // Shuffle the cell list 
    for (int i = valid_cells - 1; i > 0; i--) {
        int j = rand() % (i + 1);
        CellPos temp = cell_list[i];
        cell_list[i] = cell_list[j];
        cell_list[j] = temp;
    }
    
    for (int i = 0; i < obstacle_count; i++) {
        int x = cell_list[i].x;
        int y = cell_list[i].y;
        int z = cell_list[i].z;
        building[z][y][x].obstacle = 1;
    }
    
    free(cell_list);
}

void assign_risk_from_obstacles(const Config *cfg) {
    if (!building || !cfg) return;
    
    // Assign risk levels (0-3) based on proximity to obstacles
    // Risk 0: no nearby obstacles
    // Risk 1: 1 obstacle within 1 cell
    // Risk 2: 2+ obstacles within 1 cell or 1 obstacle at same cell
    // Risk 3: obstacle at current cell or multiple obstacles very close
    
    for (int z = 0; z < cfg->grid_z; z++) {
        for (int y = 0; y < cfg->grid_y; y++) {
            for (int x = 0; x < cfg->grid_x; x++) {
                if (building[z][y][x].obstacle) {
                    building[z][y][x].risk = 3;
                } else {
                    // Count nearby obstacles
                    int nearby_obstacles = 0;
                    for (int dz = -1; dz <= 1; dz++) {
                        for (int dy = -1; dy <= 1; dy++) {
                            for (int dx = -1; dx <= 1; dx++) {
                                if (dx == 0 && dy == 0 && dz == 0) continue;
                                
                                int nx = x + dx;
                                int ny = y + dy;
                                int nz = z + dz;
                                
                                if (nx >= 0 && nx < cfg->grid_x &&
                                    ny >= 0 && ny < cfg->grid_y &&
                                    nz >= 0 && nz < cfg->grid_z) {
                                    if (building[nz][ny][nx].obstacle) {
                                        nearby_obstacles++;
                                    }
                                }
                            }
                        }
                    }
                    
                    // Assign risk based on nearby obstacles
                    if (nearby_obstacles == 0) {
                        building[z][y][x].risk = 0;
                    } else if (nearby_obstacles == 1) {
                        building[z][y][x].risk = 1;
                    } else if (nearby_obstacles <= 3) {
                        building[z][y][x].risk = 2;
                    } else {
                        building[z][y][x].risk = 3;
                    }
                }
            }
        }
    }
}

void simulate_sensors(const Config *cfg) {
    if (!building || !cfg) return;
    
    // Simulate heat and CO2 sensors    
    for (int z = 0; z < cfg->grid_z; z++) {
        for (int y = 0; y < cfg->grid_y; y++) {
            for (int x = 0; x < cfg->grid_x; x++) {
                if (building[z][y][x].obstacle) {
                    building[z][y][x].heat = 0.0;
                    building[z][y][x].co2 = 0.0;
                    continue;
                }
                
                building[z][y][x].heat = (float)rand() / RAND_MAX;
                building[z][y][x].co2 = (float)rand() / RAND_MAX;
            }
        }
    }
}

void detect_survivors(const Config *cfg) {
    if (!building || !cfg) return;
    
    // Detect survivors based on heat and CO2 sensor thresholds
    // heat >= heat_threshold AND co2 >= co2_threshold
    // The cell is not an obstacle
    
    for (int z = 0; z < cfg->grid_z; z++) {
        for (int y = 0; y < cfg->grid_y; y++) {
            for (int x = 0; x < cfg->grid_x; x++) {
                // Reset survivor flag first
                building[z][y][x].survivor = 0;
                
                // Skip obstacle cells
                if (building[z][y][x].obstacle) {
                    continue;
                }
                
                // Check if sensor readings exceed thresholds
                if (building[z][y][x].heat >= cfg->heat_threshold &&
                    building[z][y][x].co2 >= cfg->co2_threshold) {
                    building[z][y][x].survivor = 1;
                }
            }
        }
    }
}

int count_survivors(const Config *cfg) {
    if (!building || !cfg) return 0;
    
    int count = 0;
    for (int z = 0; z < cfg->grid_z; z++) {
        for (int y = 0; y < cfg->grid_y; y++) {
            for (int x = 0; x < cfg->grid_x; x++) {
                if (building[z][y][x].survivor) {
                    count++;
                }
            }
        }
    }
    return count;
}