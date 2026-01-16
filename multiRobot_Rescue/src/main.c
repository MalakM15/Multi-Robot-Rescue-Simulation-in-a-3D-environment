#include "all_headers.h"

int calculate_astar_path_length(const RobotMission *mission, const Survivor survivors[], const Config *cfg) {
    if (mission->survivor_count == 0) return 0;
    
    int total = 0;
    Node current = mission->robot_pos;
    
    for (int s = 0; s < mission->survivor_count; s++) {
        int sid = mission->survivor_sequence[s];
        if (sid >= 0 && sid < 1000) {
            Node survivor_pos = survivors[sid].pos;
            Path path_to = astar(current, survivor_pos, cfg);
            if (path_to.valid) {
                total += path_to.length * 2;  // Round trip
            }
            current = mission->robot_pos;  // Return to base
        }
    }
    return total;
}

// Check if a cell has at least one free neighbor (so robot can move from it)
static int has_free_neighbor(Node cell, const Config *cfg) {
    if (!building || !cfg) return 0;
    
    // Check 6-directional neighbors
    int neighbor_offsets[][3] = {
        {0, 0, 1},   // up
        {0, 0, -1},  // down
        {0, 1, 0},   // north
        {0, -1, 0},  // south
        {1, 0, 0},   // east
        {-1, 0, 0}   // west
    };
    
    for (int i = 0; i < 6; i++) {
        Node neighbor = {
            cell.x + neighbor_offsets[i][0],
            cell.y + neighbor_offsets[i][1],
            cell.z + neighbor_offsets[i][2]
        };
        
        // Check if neighbor is valid 
        if (neighbor.x >= 0 && neighbor.x < cfg->grid_x &&
            neighbor.y >= 0 && neighbor.y < cfg->grid_y &&
            neighbor.z >= 0 && neighbor.z < cfg->grid_z &&
            is_valid(neighbor, cfg)) {
            return 1;  
        }
    }
    
    return 0;  // No free neighbors 
}


// Structure to store optimal solution
typedef struct {
    int assignment[10000];      // assignment[survivor] = robot_id 
    int robot_survivors[100][10000];  // robot_survivors[robot][i] = survivor_id
    int robot_survivor_count[100];
    int total_path_length;
    int survivors_rescued;
    int valid;
} OptimalSolution;

// Calculate total path length for a given assignment
static int calc_assignment_cost(int robot_count, int survivor_count, 
                                int assignment[], Node robot_starts[], 
                                Survivor survivors[], const Config *cfg) {
    int total_cost = 0;
    
    for (int r = 0; r < robot_count; r++) {
        Node current = robot_starts[r];
        
        // Find all survivors assigned to this robot
        for (int s = 0; s < survivor_count; s++) {
            if (assignment[s] == r) {
                Path p = astar(current, survivors[s].pos, cfg);
                if (p.valid) {
                    total_cost += p.length * 2; 
                } else {
                    return 999999;  // Invalid path
                }
                current = robot_starts[r];  // Return to base
            }
        }
    }
    return total_cost;
}

// Recursive function to try all assignments 
static void try_all_assignments(int survivor_idx, int survivor_count, int robot_count,
                                int current_assignment[], int *best_cost,
                                int best_assignment[], Node robot_starts[],
                                Survivor survivors[], const Config *cfg,
                                int *iterations) {
    // Limit iterations to avoid infinite loops
    if (*iterations > 1000000) return;
    (*iterations)++;
    
    if (survivor_idx == survivor_count) {
        // All survivors assigned, calculate cost
        int cost = calc_assignment_cost(robot_count, survivor_count, 
                                        current_assignment, robot_starts, 
                                        survivors, cfg);
        if (cost < *best_cost) {
            *best_cost = cost;
            for (int i = 0; i < survivor_count; i++) {
                best_assignment[i] = current_assignment[i];
            }
        }
        return;
    }
    
    // Try assigning this survivor to each robot
    for (int r = 0; r < robot_count; r++) {
        current_assignment[survivor_idx] = r;
        try_all_assignments(survivor_idx + 1, survivor_count, robot_count,
                           current_assignment, best_cost, best_assignment,
                           robot_starts, survivors, cfg, iterations);
    }
}

// Find optimal solution using A* search
static OptimalSolution find_optimal_astar_solution(int robot_count, int survivor_count,
                                                    Node robot_starts[], Survivor survivors[],
                                                    const Config *cfg) {
    OptimalSolution opt = {0};
    opt.valid = 0;
    
    // For large problems, use greedy instead 
    int max_survivors_for_brute_force = 10;  // r^s 
    
    if (survivor_count <= max_survivors_for_brute_force && robot_count <= 10) {
        // Brute force - try all possible assignments
        printf("  Computing OPTIMAL solution (exhaustive search)...              \n");
        
        int current_assignment[10000];
        int best_assignment[10000];
        int best_cost = 999999;
        int iterations = 0;
        
        for (int i = 0; i < survivor_count; i++) {
            current_assignment[i] = 0;
            best_assignment[i] = 0;
        }
        
        try_all_assignments(0, survivor_count, robot_count,
                           current_assignment, &best_cost, best_assignment,
                           robot_starts, survivors, cfg, &iterations);
                    
        printf("  Explored %d assignment combinations                            \n", iterations);
        
        if (best_cost < 999999) {
            opt.valid = 1;
            opt.total_path_length = best_cost;
            opt.survivors_rescued = survivor_count;
            
            // Store assignment
            for (int r = 0; r < robot_count; r++) {
                opt.robot_survivor_count[r] = 0;
            }
            for (int s = 0; s < survivor_count; s++) {
                opt.assignment[s] = best_assignment[s];
                int r = best_assignment[s];
                if (r >= 0 && r < 100) {
                    opt.robot_survivors[r][opt.robot_survivor_count[r]++] = s;
                }
            }
        }
    } else {
        // For larger problems, use greedy nearest-neighbor as approximation
        printf("  Problem too large for exhaustive search, using greedy approx.  \n");
        
        opt.valid = 1;
        opt.total_path_length = 0;
        opt.survivors_rescued = 0;
        
    for (int r = 0; r < robot_count; r++) {
            opt.robot_survivor_count[r] = 0;
        }
                    
        // Assign survivors to nearest available robot
        for (int s = 0; s < survivor_count; s++) {
            int best_robot = -1;
            int best_dist = 999999;
            
            for (int r = 0; r < robot_count; r++) {
                Path p = astar(robot_starts[r], survivors[s].pos, cfg);
                if (p.valid && p.length < best_dist) {
                    best_dist = p.length;
                    best_robot = r;
                }
            }
            
            if (best_robot >= 0 && best_robot < 100) {
                opt.assignment[s] = best_robot;
                opt.robot_survivors[best_robot][opt.robot_survivor_count[best_robot]++] = s;
                opt.total_path_length += best_dist * 2;
                opt.survivors_rescued++;
        }
        }
    }
    
    return opt;
}


int main(int argc, char **argv) {    
    Config cfg = {0};  // initialize all fields to 0 as a safety net
    if (load_config("configfile.txt", &cfg) != 0) {
        fprintf(stderr, "Failed to load config file.\n");
        return 1;
    }

    printf("Grid size: %d x %d x %d\n", cfg.grid_x, cfg.grid_y, cfg.grid_z);
    printf("Obstacle density: %.2f\n", cfg.obstacle_density);
    printf("Robots: %d, Population: %d, Generations: %d\n",
           cfg.robot_count, cfg.population_size, cfg.generations);

    // Allocate and initialize grid
    if (allocate_grid(&cfg) != 0) {
        fprintf(stderr, "Failed to allocate grid.\n");
        return 1;
    }
    
    // Initialize random seed
    srand(time(NULL));
    
    // Generate grid content
    generate_obstacles(&cfg);
    assign_risk_from_obstacles(&cfg);
    simulate_sensors(&cfg);
    detect_survivors(&cfg);
    
    int total_survivors = count_survivors(&cfg);
    printf("Detected %d survivors in the grid.\n", total_survivors);

    Chromosome *population = allocate_population(cfg.population_size, cfg.robot_count, cfg.max_survivors_per_robot);
    if (!population) {
        fprintf(stderr, "Failed to allocate population.\n");
        return 1;
    }

    int max_assignable_survivors = cfg.max_survivors_per_robot * cfg.robot_count;
    
    Survivor *survivors = malloc(max_assignable_survivors * sizeof(Survivor));
    if (!survivors) {
        fprintf(stderr, "Failed to allocate survivors array.\n");
        return 1;
    }
    
    int survivor_count = list_survivors(survivors, max_assignable_survivors, &cfg);
    
    if (survivor_count == 0) {
        printf("\nNo survivors detected in the grid.\n");
        printf("Cannot run rescue algorithm without survivors.\n");
        printf("Please adjust sensor thresholds or obstacle density.\n");
        free(survivors);
        free_population(population, cfg.population_size, cfg.robot_count);
        free_grid(&cfg);
        shutdown_process_pool();
        return 0;
    }

    Node robot_starts[cfg.robot_count];
    printf("\nRobot starting positions (borders at z=0):\n");
    
    for (int r = 0; r < cfg.robot_count; r++) {
        robot_starts[r].z = 0;  // All robots start at z=0
        
        // Distribute robots along the borders of the grid
        if (cfg.robot_count == 1) {
            robot_starts[r] = (Node){0, 0, 0};
        } else {
            int border_positions = 4;  
            if (r < border_positions) {
                // Place at corners
                switch (r) {
                    case 0: robot_starts[r] = (Node){0, 0, 0}; break;  // Bottom-left
                    case 1: robot_starts[r] = (Node){cfg.grid_x - 1, 0, 0}; break;  // Bottom-right
                    case 2: robot_starts[r] = (Node){0, cfg.grid_y - 1, 0}; break;  // Top-left
                    case 3: robot_starts[r] = (Node){cfg.grid_x - 1, cfg.grid_y - 1, 0}; break;  // Top-right
                }
            } else {
                // Place on edges 
                int edge_idx = r - border_positions;
                int perimeter = 2 * (cfg.grid_x + cfg.grid_y - 2);  // Perimeter length
                if (perimeter > 0) {
                    int pos = (edge_idx * perimeter) / (cfg.robot_count - border_positions + 1);
                    
                    if (pos < cfg.grid_x) {
                        // Bottom edge
                        robot_starts[r] = (Node){pos, 0, 0};
                    } else if (pos < cfg.grid_x + cfg.grid_y - 1) {
                        // Right edge
                        robot_starts[r] = (Node){cfg.grid_x - 1, pos - cfg.grid_x + 1, 0};
                    } else if (pos < 2 * cfg.grid_x + cfg.grid_y - 2) {
                        // Top edge
                        robot_starts[r] = (Node){cfg.grid_x - 1 - (pos - cfg.grid_x - cfg.grid_y + 2), cfg.grid_y - 1, 0};
                    } else {
                        // Left edge
                        robot_starts[r] = (Node){0, cfg.grid_y - 1 - (pos - 2 * cfg.grid_x - cfg.grid_y + 2), 0};
                    }
                } else {
                    robot_starts[r] = (Node){0, 0, 0};
                }
            }
        }
        
        // Ensure positions are within bounds
        if (robot_starts[r].x >= cfg.grid_x) robot_starts[r].x = cfg.grid_x - 1;
        if (robot_starts[r].y >= cfg.grid_y) robot_starts[r].y = cfg.grid_y - 1;
        if (robot_starts[r].x < 0) robot_starts[r].x = 0;
        if (robot_starts[r].y < 0) robot_starts[r].y = 0;
        
        int needs_replacement = 0;
        if (building != NULL) {
            // Check if current position is valid and has free neighbors
            if (!is_valid(robot_starts[r], &cfg)) {
                needs_replacement = 1;  // Cell is an obstacle
            } else if (!has_free_neighbor(robot_starts[r], &cfg)) {
                needs_replacement = 1;  // Cell is free but isolated 
            }
        }
        
        if (building != NULL && needs_replacement) {
            // Try nearby positions first 
            int found = 0;
            
            // Try immediate neighbors first
            int offsets[][3] = {
                {0, 1, 0}, {1, 0, 0}, {0, -1, 0}, {-1, 0, 0},  // 4-directional
                {1, 1, 0}, {1, -1, 0}, {-1, 1, 0}, {-1, -1, 0},  // Diagonal
                {0, 0, 1}, {0, 0, -1}  // Vertical
            };
            
            for (int i = 0; i < 10 && !found; i++) {
                Node test = {
                    robot_starts[r].x + offsets[i][0],
                            robot_starts[r].y + offsets[i][1], 
                    robot_starts[r].z + offsets[i][2]
                };
                if (test.x >= 0 && test.x < cfg.grid_x && 
                    test.y >= 0 && test.y < cfg.grid_y &&
                    test.z >= 0 && test.z < cfg.grid_z &&
                    is_valid(test, &cfg) &&
                    has_free_neighbor(test, &cfg)) {  // Must have at least one free neighbor
                    robot_starts[r] = test;
                    found = 1;
                }
            }
            
            // If still not found, search entire grid (prioritize z=0, then borders)
            if (!found) {
                // First try all cells at z=0
                for (int y = 0; y < cfg.grid_y && !found; y++) {
                    for (int x = 0; x < cfg.grid_x && !found; x++) {
                        Node test = {x, y, 0};
                        if (is_valid(test, &cfg) && has_free_neighbor(test, &cfg)) {
                    robot_starts[r] = test;
                    found = 1;
                }
            }
        }
        
                // If still not found, try all cells in entire grid
                if (!found) {
                    for (int z = 0; z < cfg.grid_z && !found; z++) {
                        for (int y = 0; y < cfg.grid_y && !found; y++) {
                            for (int x = 0; x < cfg.grid_x && !found; x++) {
                                Node test = {x, y, z};
                                if (is_valid(test, &cfg) && has_free_neighbor(test, &cfg)) {
                                    robot_starts[r] = test;
                                    found = 1;
                                }
                            }
                        }
                    }
                }
            }
            
            if (!found) {
                fprintf(stderr, "\nERROR: Cannot place robot %d - no free cells with free neighbors available!\n", r);
                fprintf(stderr, "All free cells are isolated (surrounded by obstacles).\n");
                fprintf(stderr, "Reduce OBSTACLE_DENSITY or increase grid size.\n");
                free(survivors);
                free_population(population, cfg.population_size, cfg.robot_count);
                free_grid(&cfg);
                shutdown_process_pool();
                return 1;
            }
        }
        
        // Verify robot is on a free cell with at least one free neighbor
        if (building != NULL) {
            if (!is_valid(robot_starts[r], &cfg)) {
                fprintf(stderr, "\nERROR: Robot %d is placed on an obstacle at (%d, %d, %d)!\n",
                       r, robot_starts[r].x, robot_starts[r].y, robot_starts[r].z);
                free(survivors);
                free_population(population, cfg.population_size, cfg.robot_count);
                free_grid(&cfg);
                shutdown_process_pool();
                return 1;
            }
            if (!has_free_neighbor(robot_starts[r], &cfg)) {
                fprintf(stderr, "\nERROR: Robot %d is placed in an isolated cell at (%d, %d, %d)!\n",
               r, robot_starts[r].x, robot_starts[r].y, robot_starts[r].z);
                fprintf(stderr, "The cell has no free neighbors - robot cannot move.\n");
                free(survivors);
                free_population(population, cfg.population_size, cfg.robot_count);
                free_grid(&cfg);
                shutdown_process_pool();
                return 1;
            }
        }
        
        printf("  Robot %d starts at: (%d, %d, %d) [FREE CELL WITH PATH]\n",
               r, robot_starts[r].x, robot_starts[r].y, robot_starts[r].z);
    }
    seed_population(population, cfg.population_size, robot_starts,
        cfg.robot_count, survivors, survivor_count, &cfg);

    // Run genetic algorithm evolution
    evolve_loop(cfg.generations, population, cfg.population_size, cfg.robot_count,
                cfg.mutation_rate, cfg.elitism_percent,
                robot_starts, survivors, survivor_count, &cfg);

    printf("\n");
    
    Chromosome best = population[0];  // Best chromosome after evolution
    
    for (int r = 0; r < cfg.robot_count; r++) {
        RobotMission *mission = &best.missions[r];
        if (mission->survivor_count > 0) {
            Node combined_path_steps[MAX_PATH_LEN];
            int combined_path_length = 0;
            
            Node current = mission->robot_pos;
            
            // Add starting position
            if (combined_path_length < MAX_PATH_LEN) {
                combined_path_steps[combined_path_length++] = current;
            }
            
            // Build survivor sequence string
            char survivor_seq[500] = "";
            int seq_pos = 0;
            
            int total_steps = 0;
            
            for (int s = 0; s < mission->survivor_count; s++) {
                int sid = mission->survivor_sequence[s];
                if (sid >= 0 && sid < survivor_count) {
                    // Add to survivor sequence string
                    if (seq_pos > 0) seq_pos += sprintf(survivor_seq + seq_pos, ",");
                    seq_pos += sprintf(survivor_seq + seq_pos, "S%d", sid);
                    
                    Node survivor_pos = survivors[sid].pos;
                    
                    // Compute path to survivor
                    Path path_to = astar(current, survivor_pos, &cfg);
                    if (path_to.valid) {
                        total_steps += path_to.length;
                        // Add path steps (skip first if same as current)
                        int start_idx = (combined_path_length > 0 &&
                                        combined_path_steps[combined_path_length - 1].x == path_to.steps[0].x &&
                                        combined_path_steps[combined_path_length - 1].y == path_to.steps[0].y &&
                                        combined_path_steps[combined_path_length - 1].z == path_to.steps[0].z) ? 1 : 0;
                        
                        for (int i = start_idx; i < path_to.length && combined_path_length < MAX_PATH_LEN; i++) {
                            combined_path_steps[combined_path_length++] = path_to.steps[i];
                        }
                        
                        // Return to base by reversing the path 
                        total_steps += path_to.length;  // Same length for return trip
                        for (int i = path_to.length - 2; i >= 0 && combined_path_length < MAX_PATH_LEN; i--) {
                            combined_path_steps[combined_path_length++] = path_to.steps[i];
                        }
                        current = mission->robot_pos;  // Reset to base for next survivor
                    }
                }
            }
            
            // Print robot header
            printf("  Robot %d: Start (%d,%d,%d) -> %s (%d steps)\n", 
                   r, mission->robot_pos.x, mission->robot_pos.y, mission->robot_pos.z, 
                   survivor_seq, total_steps);
            
            const int max_steps_to_show = 50;
            int steps_to_show = (combined_path_length > max_steps_to_show) ? max_steps_to_show : combined_path_length;
            
            printf("\n    Path: ");
            const int chars_per_line = 75;  // Characters per line
            int line_chars = 10;  // "    Path: "
            
            for (int i = 0; i < steps_to_show; i++) {
                char step_str[30];
                sprintf(step_str, "(%d,%d,%d)", 
                        combined_path_steps[i].x, 
                        combined_path_steps[i].y, 
                        combined_path_steps[i].z);
                
                if (i < steps_to_show - 1 || combined_path_length > max_steps_to_show) {
                    sprintf(step_str + strlen(step_str), "->");
                }
                
                int step_len = strlen(step_str);
                
                // Check if we need a line break
                if (line_chars + step_len > chars_per_line && i > 0) {
                    printf("\n          ");
                    line_chars = 10;  // "          " is 10 chars
                }
                
                printf("%s", step_str);
                line_chars += step_len;
            }
            
            // Show remaining count if path was truncated
            if (combined_path_length > max_steps_to_show) {
                int remaining = combined_path_length - max_steps_to_show;
                if (line_chars + 15 > chars_per_line) {
                    printf("\n          ");
                }
                printf("... (+%d more)", remaining);
            }
            
            printf("\n");
        }
    }
    printf("\n");

    printf("\n");
    printf("+============================================================================+\n");
    printf("|                      OPTIMAL A* vs GA COMPARISON                           |\n");
    printf("+============================================================================+\n");
    
    OptimalSolution opt = find_optimal_astar_solution(cfg.robot_count, survivor_count,
                                                       robot_starts, survivors, &cfg);
    
    printf("\n");
    printf("  [1] OPTIMAL A* SOLUTION                                                   \n");
    printf("  ------------------------                                                  \n");
    printf("  Total Path Length: %-5d steps                                            \n", opt.total_path_length);
    printf("  Survivors Rescued: %-3d                                                    \n", opt.survivors_rescued);
    printf("                                                                            \n");
    printf("  Robot Assignments:                                                        \n");
    
    for (int r = 0; r < cfg.robot_count; r++) {
        if (opt.robot_survivor_count[r] > 0) {
            char survivors_str[500] = "";
            int pos = 0;
            for (int i = 0; i < opt.robot_survivor_count[r] && i < 10; i++) {
                pos += sprintf(survivors_str + pos, "S%d", opt.robot_survivors[r][i]);
                if (i < opt.robot_survivor_count[r] - 1 && i < 9) {
                    pos += sprintf(survivors_str + pos, ",");
                }
            }
            if (opt.robot_survivor_count[r] > 10) {
                pos += sprintf(survivors_str + pos, "...(+%d)", opt.robot_survivor_count[r] - 10);
            }
            printf("    Robot %2d: %-50s\n", r, survivors_str);
        }
    }
    
    printf("\n");
    printf("  [2] Genetic Algorithm SOLUTION      (%d generations)                      \n", cfg.generations);
    printf("  -----------------------------------------                                 \n");
    
    
    int ga_total_path = 0;
    int ga_survivors_rescued = 0;
    
    // GA metrics from best chromosome
    for (int r = 0; r < cfg.robot_count; r++) {
        RobotMission *mission = &best.missions[r];
        if (mission->survivor_count > 0) {
            ga_survivors_rescued += mission->survivor_count;
            ga_total_path += calculate_astar_path_length(mission, survivors, &cfg);
        }
    }
    
    printf("  Total Path Length: %-5d steps                                            \n", ga_total_path);
    printf("  Survivors Rescued: %-3d                                                    \n", ga_survivors_rescued);
    printf("  Fitness Score:     %-10.2f                                              \n", best.fitness);
    printf("                                                                            \n");
    printf("  Robot Assignments:                                                        \n");
    
    for (int r = 0; r < cfg.robot_count; r++) {
        RobotMission *mission = &best.missions[r];
        
        if (mission->survivor_count > 0) {
            char survivors_str[500] = "";
            int pos = 0;
            
            for (int s = 0; s < mission->survivor_count && s < 10; s++) {
                int sid = mission->survivor_sequence[s];
                if (sid >= 0) {
                    pos += sprintf(survivors_str + pos, "S%d", sid);
                    if (s < mission->survivor_count - 1 && s < 9) {
                        pos += sprintf(survivors_str + pos, ",");
                    }
                }
            }
            if (mission->survivor_count > 10) {
                pos += sprintf(survivors_str + pos, "...(+%d)", mission->survivor_count - 10);
            }
            
            printf("    Robot %2d: %-50s\n", r, survivors_str);
        }
    }
    
    printf("                                                                            \n");
    printf("+============================================================================+\n");

    // ============ OPENGL VISUALIZATION ============
    printf("\nPreparing 3D visualization...\n");
        
        // Compute paths for the best GA solution
        Path *robot_paths = calloc(cfg.robot_count, sizeof(Path));
        if (robot_paths) {
            for (int r = 0; r < cfg.robot_count; r++) {
                RobotMission *mission = &best.missions[r];
                if (mission->survivor_count > 0) {
                    Path combined_path = {0};
                    combined_path.valid = 1;
                    
                    Node current = mission->robot_pos;
                    int path_idx = 0;
                    
                    // Add starting position
                    if (path_idx < MAX_PATH_LEN) {
                        combined_path.steps[path_idx++] = current;
                    }
                    
                    for (int s = 0; s < mission->survivor_count && path_idx < MAX_PATH_LEN - 1; s++) {
                        int sid = mission->survivor_sequence[s];
                        if (sid >= 0 && sid < survivor_count) {
                            Node survivor_pos = survivors[sid].pos;
                            Path path_to_survivor = astar(current, survivor_pos, &cfg);
                            
                            if (path_to_survivor.valid) {
                                for (int i = 1; i < path_to_survivor.length && path_idx < MAX_PATH_LEN; i++) {
                                    combined_path.steps[path_idx++] = path_to_survivor.steps[i];
                                }
                                
                                for (int i = path_to_survivor.length - 2; i >= 0 && path_idx < MAX_PATH_LEN; i--) {
                                    combined_path.steps[path_idx++] = path_to_survivor.steps[i];
                                }
                                current = mission->robot_pos;
                            }
                        }
                    }
                    
                    combined_path.length = path_idx;
                    robot_paths[r] = combined_path;
                } else {
                    // No survivors assigned to this robot
                    robot_paths[r].valid = 0;
                    robot_paths[r].length = 0;
                }
            }
            
            // Initialize visualization with the best solution
                init_visualization(&cfg, robot_paths, cfg.robot_count, 
                                  survivors, survivor_count, robot_starts);
                
                // Start visualization 
                start_visualization(argc, argv);
            
            
            free(robot_paths);
        } else {
            printf("Warning: Failed to allocate memory for visualization paths.\n");
        }

    free_population(population, cfg.population_size, cfg.robot_count);
    free(survivors);  
    free_grid(&cfg);
    
    
    extern void shutdown_process_pool(void);
    shutdown_process_pool();
    
   
    signal(SIGINT, SIG_DFL);
    signal(SIGTERM, SIG_DFL);
    signal(SIGSEGV, SIG_DFL);
    signal(SIGABRT, SIG_DFL);
    signal(SIGCHLD, SIG_DFL);
    
    pid_t pid;
    int status;
    while ((pid = waitpid(-1, &status, WNOHANG)) > 0) {
    }
    
    return 0;
}
