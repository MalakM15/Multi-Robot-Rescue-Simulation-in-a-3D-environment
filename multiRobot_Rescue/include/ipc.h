#ifndef IPC_H
#define IPC_H

#include "grid.h"
#include "ga.h"
#include "config.h"

int ipc_init_shared(const Config *cfg);
void ipc_attach_grid(Cell ***grid_ptr);
void ipc_attach_population(Chromosome **pop_ptr, int pop_size, int robot_count);
void ipc_attach_fitness(double **fitness_ptr, int pop_size);

void ipc_spawn_workers(int worker_count);
void ipc_distribute_jobs(int pop_size, int robot_count);
void ipc_collect_fitness(double fitness_out[], int pop_size);

void ipc_cleanup();

#endif
