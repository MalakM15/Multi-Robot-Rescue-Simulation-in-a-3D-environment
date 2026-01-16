#include "all_headers.h"

Chromosome *allocate_population(int pop_size, int robot_count, int max_survivors_per_robot) {
    Chromosome *pop = malloc(pop_size * sizeof(Chromosome));
    if (!pop) return NULL;

    for (int i = 0; i < pop_size; i++) {
        pop[i].missions = malloc(robot_count * sizeof(RobotMission));
        if (!pop[i].missions) {
            // Free previously allocated missions and pop
            for (int j = 0; j < i; j++) {
                for (int r = 0; r < robot_count; r++) {
                    free(pop[j].missions[r].survivor_sequence);
                }
                free(pop[j].missions);
            }
            free(pop);
            return NULL;
        }

        // initialize fitness and missions
        pop[i].fitness = 0.0;
        for (int r = 0; r < robot_count; r++) {
            pop[i].missions[r].robot_pos = (Node){0, 0, 0};  // Default start
            pop[i].missions[r].survivor_count = 0;  // No survivors assigned yet
            pop[i].missions[r].survivor_sequence = malloc(max_survivors_per_robot * sizeof(int));
            if (!pop[i].missions[r].survivor_sequence) {
                // Free previously allocated arrays
                for (int rr = 0; rr < r; rr++) {
                    free(pop[i].missions[rr].survivor_sequence);
                }
                free(pop[i].missions);
                for (int j = 0; j < i; j++) {
                    for (int rr = 0; rr < robot_count; rr++) {
                        free(pop[j].missions[rr].survivor_sequence);
                    }
                    free(pop[j].missions);
                }
                free(pop);
                return NULL;
            }
            // Initialize all slots to -1 (unassigned)
            for (int s = 0; s < max_survivors_per_robot; s++) {
                pop[i].missions[r].survivor_sequence[s] = -1;
            }
        }
    }

    return pop;
}

void free_population(Chromosome *pop, int pop_size, int robot_count) {
    if (!pop) return;

    for (int i = 0; i < pop_size; i++) {
        if (pop[i].missions) {
            // Free survivor_sequence arrays for each robot
            for (int r = 0; r < robot_count; r++) {
                free(pop[i].missions[r].survivor_sequence);
            }
            free(pop[i].missions);
        }
    }
    free(pop);
}

void seed_population(Chromosome pop[], int pop_size, const Node robot_starts[], int robot_count, const Survivor survivors[], int survivor_count, const Config *cfg) { 
    (void)survivors; 
    (void)cfg;      
    if (survivor_count == 0) {
        printf("Warning: No survivors detected. All robots will be unassigned.\n");
        // Initialize all missions 
        int max_per_robot = cfg->max_survivors_per_robot;
        for (int i = 0; i < pop_size; i++) {
            for (int r = 0; r < robot_count; r++) {
                pop[i].missions[r].robot_pos = robot_starts ? robot_starts[r] : (Node){0, 0, 0};
                pop[i].missions[r].survivor_count = 0;
                for (int s = 0; s < max_per_robot; s++) {
                    pop[i].missions[r].survivor_sequence[s] = -1;
                }
            }
            pop[i].fitness = 0.0;
        }
        return;
    }
    if (robot_count == 0) {
        printf("Error: enter robot_count more than 0\n");
        return;
    }
    if (pop_size == 0) {
        printf("Error: enter Population size more than 0\n");
        return;
    }

    for (int i = 0; i < pop_size; i++) {
       
        // Create a list of all survivor IDs
        int available_survivors[survivor_count];
        for (int s = 0; s < survivor_count; s++) {
            available_survivors[s] = s;
        }
        
        // Shuffle the list for each chromosome
        for (int s = survivor_count - 1; s > 0; s--) {
            int j = rand() % (s + 1);
            int temp = available_survivors[s];
            available_survivors[s] = available_survivors[j];
            available_survivors[j] = temp;
        }

        //  ensure each robot gets at least one survivor
        int survivors_per_robot_base = survivor_count / robot_count;
        int extra_survivors = survivor_count % robot_count;
        int survivor_idx = 0;
        
        for (int r = 0; r < robot_count; r++) {
            RobotMission *mission = &pop[i].missions[r];
            mission->robot_pos = robot_starts ? robot_starts[r] : (Node){0, 0, 0};
            
            // Each robot gets base number of survivors and extra if available
            int count = survivors_per_robot_base + (r < extra_survivors ? 1 : 0);
            int max_per_robot = cfg->max_survivors_per_robot;
            if (count > max_per_robot) count = max_per_robot;
            
            // Assign survivors in round-robin fashion
            for (int s = 0; s < count && survivor_idx < survivor_count; s++) {
                mission->survivor_sequence[s] = available_survivors[survivor_idx++];
            }
            
            mission->survivor_count = count;
            
            // Fill remaining slots with -1
            for (int s = count; s < max_per_robot; s++) {
                mission->survivor_sequence[s] = -1;
            }
        }
        
        // distribute extra servivors to robots with fewer assignments
        int max_per_robot_limit = cfg->max_survivors_per_robot;
        while (survivor_idx < survivor_count) {
            // Find robot with fewest assignments
            int min_robot = 0;
            int min_count = pop[i].missions[0].survivor_count;
            for (int r = 1; r < robot_count; r++) {
                if (pop[i].missions[r].survivor_count < min_count) {
                    min_count = pop[i].missions[r].survivor_count;
                    min_robot = r;
                }
            }
            
            if (min_count < max_per_robot_limit) {
                pop[i].missions[min_robot].survivor_sequence[min_count] = available_survivors[survivor_idx++];
                pop[i].missions[min_robot].survivor_count++;
            } else {
                break;
            }
        }

        pop[i].fitness = 0.0; 
    }
}

static double manhattan_distance(Node a, Node b) {
    return abs(a.x - b.x) + abs(a.y - b.y) + abs(a.z - b.z);
}

static double euclidean_distance(Node a, Node b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = a.z - b.z;
    return sqrt(dx*dx + dy*dy + dz*dz);
}

static double fast_path_cost(Node start, Node end, const Config *cfg) {
    if (building == NULL) {
        return manhattan_distance(start, end);
    }
    
    // Use Euclidean distance as base
    double base_distance = euclidean_distance(start, end);
    if (base_distance < 0.1) return base_distance; // Same node
    
    // Sample the path for obstacles and risk
    // Sample more points for longer paths to maintain accuracy
    int samples = (int)(base_distance * 2) + 3; // At least 3 samples
    if (samples > 50) samples = 50;
    
    double obstacle_penalty = 0.0;
    double risk_sum = 0.0;
    int obstacle_count = 0;
    int total_cells = 0;
    
    // Sample points along the straight-line path
    for (int i = 0; i <= samples; i++) {
        double t = (double)i / samples;
        int x = (int)(start.x + t * (end.x - start.x) + 0.5);
        int y = (int)(start.y + t * (end.y - start.y) + 0.5);
        int z = (int)(start.z + t * (end.z - start.z) + 0.5);
        
        if (x < 0) x = 0;
        if (x >= cfg->grid_x) x = cfg->grid_x - 1;
        if (y < 0) y = 0;
        if (y >= cfg->grid_y) y = cfg->grid_y - 1;
        if (z < 0) z = 0;
        if (z >= cfg->grid_z) z = cfg->grid_z - 1;
        
        total_cells++;
        if (building[z][y][x].obstacle) {
            obstacle_count++;
            obstacle_penalty += 2.0; // Penalty for each obstacle
        }
        risk_sum += building[z][y][x].risk;
    }
    
    double obstacle_density = (double)obstacle_count / total_cells;
    double detour_multiplier = 1.0 + (obstacle_density * 0.8); //  80% extra distance for high obstacle density
    
    // Calculate average risk
    double avg_risk = risk_sum / total_cells;
    
    // Estimated cost = base distance * detour_multiplier + obstacle penalties + risk factor
    double estimated_cost = base_distance * detour_multiplier + obstacle_penalty * 0.5 + avg_risk * 0.3;
    
    return estimated_cost;
}

double fitness_chromosome(const Chromosome *c, int robot_count, const Config *cfg) {
    // Fitness weights
    const double w1 = 100.0;   // Weight for unique survivors rescued
    const double w2 = 200.0;   // Bonus if all survivors are assigned
    const double w3 = 0.5;     // Weight for path length 
    const double w4 = 5.0;     // Weight for risk exposure (penalty)
    const double w5 = 50.0;    // Weight for collisions (penalty)
    const double w6 = 150.0;   // Weight for duplicate assignments (penalty)
    const double w7 = 200.0;   // Weight for valid paths 
    const double w8 = 150.0;   // Bonus if all robots have at least one survivor
    
    Survivor survivors[1000]; 
    int survivor_count = list_survivors(survivors, 1000, cfg);
    
    // unique survivors rescued
    int unique_survivors = 0;
    int max_survivor_id = -1;
    int survivor_tracked[100] = {0};  // Track which survivors are rescued
    
    // find all survivors assigned to robots
    for (int r = 0; r < robot_count; r++) {
        RobotMission *mission = &c->missions[r];
        for (int s = 0; s < mission->survivor_count; s++) {
            int sid = mission->survivor_sequence[s];
            if (sid >= 0 && sid < survivor_count) {
                if (sid > max_survivor_id) {
                    max_survivor_id = sid;
                }
                if (!survivor_tracked[sid]) {
                    survivor_tracked[sid] = 1;
                    unique_survivors++;
                }
            }
        }
    }
    
    //Calculate total path length and risk using FAST HEURISTIC
    double total_length = 0.0;
    double total_risk = 0.0;
    int valid_paths = 0;
    
    for (int r = 0; r < robot_count; r++) {
        RobotMission *mission = &c->missions[r];
        Node current_pos = mission->robot_pos;  // Start at robot position
        
        // For each survivor in sequence
        for (int s = 0; s < mission->survivor_count; s++) {
            int sid = mission->survivor_sequence[s];
            if (sid >= 0 && sid < survivor_count) {
                Node survivor_pos = survivors[sid].pos;
                
                double outbound_cost = fast_path_cost(current_pos, survivor_pos, cfg);
                double return_cost = fast_path_cost(survivor_pos, mission->robot_pos, cfg);
                
                // Check if path is valid 
                // If start and end are both valid cells, assume path exists
                int valid = 1;
                if (building != NULL) {
                    // if start or end is obstacle, path is invalid
                    if (current_pos.x >= 0 && current_pos.x < cfg->grid_x &&
                        current_pos.y >= 0 && current_pos.y < cfg->grid_y &&
                        current_pos.z >= 0 && current_pos.z < cfg->grid_z) {
                        if (building[current_pos.z][current_pos.y][current_pos.x].obstacle) {
                            valid = 0;
                        }
                    }
                    if (survivor_pos.x >= 0 && survivor_pos.x < cfg->grid_x &&
                        survivor_pos.y >= 0 && survivor_pos.y < cfg->grid_y &&
                        survivor_pos.z >= 0 && survivor_pos.z < cfg->grid_z) {
                        if (building[survivor_pos.z][survivor_pos.y][survivor_pos.x].obstacle) {
                            valid = 0;
                        }
                    }
                }
                
                if (valid) {
                    valid_paths++;
                    total_length += outbound_cost + return_cost;
                    
                    // Estimate risk: sample a few points along the path
                    if (building != NULL) {
                        int samples = 10;
                        for (int i = 0; i <= samples; i++) {
                            double t = (double)i / samples;
                            int x1 = (int)(current_pos.x + t * (survivor_pos.x - current_pos.x) + 0.5);
                            int y1 = (int)(current_pos.y + t * (survivor_pos.y - current_pos.y) + 0.5);
                            int z1 = (int)(current_pos.z + t * (survivor_pos.z - current_pos.z) + 0.5);
                            int x2 = (int)(survivor_pos.x + t * (mission->robot_pos.x - survivor_pos.x) + 0.5);
                            int y2 = (int)(survivor_pos.y + t * (mission->robot_pos.y - survivor_pos.y) + 0.5);
                            int z2 = (int)(survivor_pos.z + t * (mission->robot_pos.z - survivor_pos.z) + 0.5);
                            
                            // Clamp and sample
                            if (x1 >= 0 && x1 < cfg->grid_x && y1 >= 0 && y1 < cfg->grid_y && 
                                z1 >= 0 && z1 < cfg->grid_z) {
                                total_risk += building[z1][y1][x1].risk;
                            }
                            if (x2 >= 0 && x2 < cfg->grid_x && y2 >= 0 && y2 < cfg->grid_y && 
                                z2 >= 0 && z2 < cfg->grid_z) {
                                total_risk += building[z2][y2][x2].risk;
                            }
                        }
                    }
                    
                    current_pos = mission->robot_pos;
                } else {
                    // Invalid path use large penalty
                    total_length += manhattan_distance(current_pos, survivor_pos) * 3.0;
                }
            }
        }
    }
    
    int collisions = detect_collisions(c, robot_count);
    
    // Calculate duplicate survivor assignments 
    int duplicate_assignments = 0;
    if (max_survivor_id >= 0) {
        int *survivor_assignment_count = calloc(max_survivor_id + 1, sizeof(int));
        if (survivor_assignment_count) {
            for (int r = 0; r < robot_count; r++) {
                RobotMission *mission = &c->missions[r];
                for (int s = 0; s < mission->survivor_count; s++) {
                    int sid = mission->survivor_sequence[s];
                    if (sid >= 0 && sid <= max_survivor_id) {
                        survivor_assignment_count[sid]++;
                    }
                }
            }
            for (int i = 0; i <= max_survivor_id; i++) {
                if (survivor_assignment_count[i] > 1) {
                    duplicate_assignments += (survivor_assignment_count[i] - 1);
                }
            }
            free(survivor_assignment_count);
        }
    }
    
    // Count robots with assignments
    int active_robots = 0;
    for (int r = 0; r < robot_count; r++) {
        if (c->missions[r].survivor_count > 0) {
            active_robots++;
        }
    }
    
    // Calculate maximum assignable survivors
    int max_assignable = cfg->max_survivors_per_robot * robot_count;
    int target_survivors = (survivor_count < max_assignable) ? survivor_count : max_assignable;
    
    double all_survivors_bonus = (unique_survivors >= target_survivors) ? w2 : 0.0;
    double all_robots_bonus = (active_robots == robot_count) ? w8 : 0.0;
    
  
    // f = w1*unique_survivors + w2*all_survivors_bonus - w3*length - w4*risk - w5*collisions - w6*duplicates + w7*valid_paths + w8*all_robots_bonus
    double fitness = w1 * unique_survivors + 
                     w2 * all_survivors_bonus -
                     w3 * total_length - 
                     w4 * total_risk - 
                     w5 * collisions - 
                     w6 * duplicate_assignments +
                     w7 * valid_paths +
                     all_robots_bonus;
    
    return fitness;
}

int detect_collisions(const Chromosome *c, int robot_count) {
    if (!c) return 0;
    
    int collisions = 0;
    
    for (int r1 = 0; r1 < robot_count; r1++) {
        Node pos1 = c->missions[r1].robot_pos;
        
        for (int r2 = r1 + 1; r2 < robot_count; r2++) {
            Node pos2 = c->missions[r2].robot_pos;
            
            if (pos1.x == pos2.x && pos1.y == pos2.y && pos1.z == pos2.z) {
                collisions++;
            }
        }
    }
    
    return collisions;
}

void tournament_select(const Chromosome pop[], int pop_size,
                       Chromosome parents[], int parent_count, int robot_count) {
    if (!pop || !parents || pop_size <= 0 || parent_count <= 0) return;
    
    const int tournament_size = 3;  
    
    for (int i = 0; i < parent_count; i++) {

        int best_idx = rand() % pop_size;
        double best_fitness = pop[best_idx].fitness;
        
        for (int j = 1; j < tournament_size; j++) {
            int candidate_idx = rand() % pop_size;
            if (pop[candidate_idx].fitness > best_fitness) {
                best_idx = candidate_idx;
                best_fitness = pop[candidate_idx].fitness;
            }
        }
        
        if (parents[i].missions && pop[best_idx].missions) {
            for (int r = 0; r < robot_count; r++) {
                parents[i].missions[r] = pop[best_idx].missions[r];
            }
            parents[i].fitness = pop[best_idx].fitness;
        }
    }
}

// Repair chromosome to remove duplicate survivor assignments
static void repair_chromosome(Chromosome *c, int robot_count, int survivor_count, int max_survivors_per_robot) {
    if (!c || survivor_count <= 0) return;
    
    // Track which survivors have been assigned
    int *assigned = calloc(survivor_count, sizeof(int));
    if (!assigned) return;
    
    for (int r = 0; r < robot_count; r++) {
        RobotMission *mission = &c->missions[r];
        int new_count = 0;
        
        for (int s = 0; s < mission->survivor_count; s++) {
            int sid = mission->survivor_sequence[s];
            if (sid >= 0 && sid < survivor_count && !assigned[sid]) {
                assigned[sid] = 1;
                mission->survivor_sequence[new_count++] = sid;
            }
        }
        
        mission->survivor_count = new_count;
        for (int s = new_count; s < max_survivors_per_robot; s++) {
            mission->survivor_sequence[s] = -1;
        }
    }
    
    //  assign unassigned survivors to robots with fewest assignments
    for (int sid = 0; sid < survivor_count; sid++) {
        if (!assigned[sid]) {
            // Find robot with fewest survivors
            int min_robot = 0;
            int min_count = c->missions[0].survivor_count;
            for (int r = 1; r < robot_count; r++) {
                if (c->missions[r].survivor_count < min_count) {
                    min_count = c->missions[r].survivor_count;
                    min_robot = r;
                }
            }
            
            if (min_count < max_survivors_per_robot) {
                c->missions[min_robot].survivor_sequence[min_count] = sid;
                c->missions[min_robot].survivor_count++;
                assigned[sid] = 1;
            }
        }
    }
    
    free(assigned);
}

void crossover(Chromosome *child, const Chromosome *p1, const Chromosome *p2,
               int robot_count, int survivor_count, int max_survivors_per_robot) {
    if (!child || !p1 || !p2) return;
    
    // randomly choose a crossover point
    int crossover_point = rand() % robot_count;
    
  
    for (int r = 0; r < robot_count; r++) {
        RobotMission *child_mission = &child->missions[r];
        const RobotMission *parent_mission = (r < crossover_point) ? &p1->missions[r] : &p2->missions[r];
        
        child_mission->robot_pos = parent_mission->robot_pos;
        child_mission->survivor_count = parent_mission->survivor_count;
        
        for (int s = 0; s < max_survivors_per_robot; s++) {
            child_mission->survivor_sequence[s] = parent_mission->survivor_sequence[s];
        }
    }
    
    // Repair to remove duplicate survivor assignments
    repair_chromosome(child, robot_count, survivor_count, max_survivors_per_robot);
    
    // Reset fitness 
    child->fitness = 0.0;
}

void mutate(Chromosome *c, int robot_count, double rate, 
            const Node robot_starts[], const Survivor survivors[], 
            int survivor_count, const Config *cfg) {
    if (!c || !cfg || survivor_count == 0) return;
    
    (void)robot_starts;
    (void)survivors;  
    int mutated = 0;
    
    for (int r = 0; r < robot_count; r++) {
        // Random mutation decision
        if ((double)rand() / RAND_MAX < rate) {
            RobotMission *mission = &c->missions[r];
            mutated = 1;
            
            // Change the sequence of survivors
            if (mission->survivor_count > 0) {
                int mutation_type = rand() % 4;
                
                if (mutation_type == 0 && mission->survivor_count > 1) {
                    // Swap two survivors in this robot's sequence
                    int idx1 = rand() % mission->survivor_count;
                    int idx2 = rand() % mission->survivor_count;
                    int temp = mission->survivor_sequence[idx1];
                    mission->survivor_sequence[idx1] = mission->survivor_sequence[idx2];
                    mission->survivor_sequence[idx2] = temp;
                } else if (mutation_type == 1 && mission->survivor_count > 1) {
                    // Remove a random survivor from sequence
                    int remove_idx = rand() % mission->survivor_count;
                    for (int i = remove_idx; i < mission->survivor_count - 1; i++) {
                        mission->survivor_sequence[i] = mission->survivor_sequence[i + 1];
                    }
                    mission->survivor_sequence[mission->survivor_count - 1] = -1;
                    mission->survivor_count--;
                } else if (mutation_type == 2 && robot_count > 1) {
                    // Move a survivor to another robot
                    int other_robot = rand() % robot_count;
                    while (other_robot == r) other_robot = rand() % robot_count;
                    
                    RobotMission *other = &c->missions[other_robot];
                    // Check limit
                    if (mission->survivor_count > 0 && other->survivor_count < cfg->max_survivors_per_robot) {
                        // Move last survivor from this robot to other
                        int sid = mission->survivor_sequence[mission->survivor_count - 1];
                        mission->survivor_sequence[mission->survivor_count - 1] = -1;
                        mission->survivor_count--;
                        other->survivor_sequence[other->survivor_count] = sid;
                        other->survivor_count++;
                    }
                } else if (mutation_type == 3 && mission->survivor_count > 1) {
                    // Reverse a portion of the sequence
                    int start = rand() % mission->survivor_count;
                    int end = rand() % mission->survivor_count;
                    if (start > end) { int t = start; start = end; end = t; }
                    while (start < end) {
                        int temp = mission->survivor_sequence[start];
                        mission->survivor_sequence[start] = mission->survivor_sequence[end];
                        mission->survivor_sequence[end] = temp;
                        start++; end--;
                    }
                }
            }
        }
    }
    
    // Repair chromosome if any mutation occurred
    if (mutated) {
        repair_chromosome(c, robot_count, survivor_count, cfg->max_survivors_per_robot);
    }
    
    // Reset fitness
    c->fitness = 0.0;
}

void sort_by_fitness(Chromosome pop[], int pop_size) {
    if (!pop || pop_size <= 0) return;
    
    // Simple bubble sort
    for (int i = 0; i < pop_size - 1; i++) {
        for (int j = 0; j < pop_size - i - 1; j++) {
            if (pop[j].fitness < pop[j + 1].fitness) {
                Chromosome temp = pop[j];
                pop[j] = pop[j + 1];
                pop[j + 1] = temp;
            }
        }
    }
}

void evolve_loop(int generations, Chromosome pop[], int pop_size,
                 int robot_count, double mutation_rate, int elitism_pct,
                 const Node robot_starts[], const Survivor survivors[],
                 int survivor_count, const Config *cfg) {
    if (!pop || pop_size <= 0 || robot_count <= 0 || !cfg) return;
    
    // Calculate number of elite individuals to preserve
    int elite_count = (pop_size * elitism_pct) / 100;
    if (elite_count < 1) elite_count = 1;
    if (elite_count >= pop_size) elite_count = pop_size - 1;
    
    Chromosome *new_pop = allocate_population(pop_size, robot_count, cfg->max_survivors_per_robot);
    Chromosome *parents = allocate_population(pop_size, robot_count, cfg->max_survivors_per_robot);
    
    if (!new_pop || !parents) {
        fprintf(stderr, "Failed to allocate memory for evolution\n");
        if (new_pop) free_population(new_pop, pop_size, robot_count);
        if (parents) free_population(parents, pop_size, robot_count);
        return;
    }
    
    printf("Starting evolution for %d generations...\n", generations);
    printf("Elite count: %d, Mutation rate: %.2f\n", elite_count, mutation_rate);
    
    extern void compute_fitness_parallel_mp(Chromosome[], int, int, const Config *,
                                             const Node[], const Survivor[], int);
    compute_fitness_parallel_mp(pop, pop_size, robot_count, cfg, 
                                robot_starts, survivors, survivor_count);
    sort_by_fitness(pop, pop_size);
    
    double best_fitness = pop[0].fitness;
    printf("Generation 0: Best fitness = %.2f\n", best_fitness);
    
    // Evolution loop
    for (int gen = 1; gen <= generations; gen++) {
        // Preserve elite individuals
        for (int i = 0; i < elite_count; i++) {
            // Deep copy elite chromosome
            for (int r = 0; r < robot_count; r++) {
                RobotMission *new_mission = &new_pop[i].missions[r];
                const RobotMission *old_mission = &pop[i].missions[r];
                
                new_mission->robot_pos = old_mission->robot_pos;
                new_mission->survivor_count = old_mission->survivor_count;
                
                for (int s = 0; s < cfg->max_survivors_per_robot; s++) {
                    new_mission->survivor_sequence[s] = old_mission->survivor_sequence[s];
                }
            }
            new_pop[i].fitness = pop[i].fitness;
        }
        
        // Generate rest of population through selection, crossover, and mutation
        for (int i = elite_count; i < pop_size; i++) {
            // Tournament selection to choose parents
            tournament_select(pop, pop_size, parents, 2, robot_count);
            
            // Crossover to create child 
            crossover(&new_pop[i], &parents[0], &parents[1], robot_count, survivor_count, cfg->max_survivors_per_robot);
            
            // Mutate child
            mutate(&new_pop[i], robot_count, mutation_rate, 
                   robot_starts, survivors, survivor_count, cfg);
        }
        
        // Compute fitness for new generation using multiprocessing
        extern void compute_fitness_parallel_mp(Chromosome[], int, int, const Config *,
                                                 const Node[], const Survivor[], int);
        compute_fitness_parallel_mp(new_pop, pop_size, robot_count, cfg,
                                    robot_starts, survivors, survivor_count);
        
        // Sort by fitness
        sort_by_fitness(new_pop, pop_size);
        
        // Replace old population with new population
        for (int i = 0; i < pop_size; i++) {
            for (int r = 0; r < robot_count; r++) {
                RobotMission *pop_mission = &pop[i].missions[r];
                const RobotMission *new_mission = &new_pop[i].missions[r];
                
                // Copy non-pointer fields
                pop_mission->robot_pos = new_mission->robot_pos;
                pop_mission->survivor_count = new_mission->survivor_count;
                
                for (int s = 0; s < cfg->max_survivors_per_robot; s++) {
                    pop_mission->survivor_sequence[s] = new_mission->survivor_sequence[s];
                }
            }
            pop[i].fitness = new_pop[i].fitness;
        }
        
        // Report progress 
        double current_best = pop[0].fitness;
        double improvement = current_best - best_fitness;
        if (gen % 25 == 0 || (improvement > 5.0 && current_best > best_fitness)) {
            printf("Generation %d: Best fitness = %.2f", gen, current_best);
            if (improvement > 0.01) {
                printf(" (improvement: %.2f)", improvement);
            }
            printf("\n");
            best_fitness = current_best;
        }
    }
    
    printf("Evolution complete. Final best fitness = %.2f\n", pop[0].fitness);
    
    // Cleanup
    free_population(new_pop, pop_size, robot_count);
    free_population(parents, pop_size, robot_count);
}