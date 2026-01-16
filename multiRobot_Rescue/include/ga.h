
#ifndef GA_H
#define GA_H

#include "astar.h"
#include "config.h"


// Robot mission structure - stores sequence of survivors to rescue
typedef struct {
    Node robot_pos;                    // Robot's starting position
    int *survivor_sequence;            // Dynamically allocated sequence of survivor IDs to rescue
    int survivor_count;                // Number of survivors assigned (limited by cfg->max_survivors_per_robot at runtime)
} RobotMission;

typedef struct {
    RobotMission *missions;  // dynamically allocated: one per robot
    double fitness;
} Chromosome;

Chromosome *allocate_population(int pop_size, int robot_count, int max_survivors_per_robot);
void free_population(Chromosome *pop, int pop_size, int robot_count);

void seed_population(Chromosome pop[], int pop_size,
                     const Node robot_starts[], int robot_count,
                     const Survivor survivors[], int survivor_count,
                     const Config *cfg);

double fitness_chromosome(const Chromosome *c, int robot_count, const Config *cfg);
int detect_collisions(const Chromosome *c, int robot_count);

void compute_fitness_parallel_mp(Chromosome pop[], int pop_size, int robot_count, const Config *cfg,
                                  const Node robot_starts[], const Survivor survivors[], int survivor_count);
int init_process_pool(int max_pop_size, int robot_count, int max_survivor_count, int pool_size, int max_survivors_per_robot);
void shutdown_process_pool(void);

void tournament_select(const Chromosome pop[], int pop_size,
                       Chromosome parents[], int parent_count, int robot_count);

void crossover(Chromosome *child, const Chromosome *p1, const Chromosome *p2,
               int robot_count, int survivor_count, int max_survivors_per_robot);

void mutate(Chromosome *c, int robot_count, double rate, 
            const Node robot_starts[], const Survivor survivors[], 
            int survivor_count, const Config *cfg);

void sort_by_fitness(Chromosome pop[], int pop_size);

void evolve_loop(int generations, Chromosome pop[], int pop_size,
                 int robot_count, double mutation_rate, int elitism_pct,
                 const Node robot_starts[], const Survivor survivors[],
                 int survivor_count, const Config *cfg);

#endif