#ifndef ASTAR_H
#define ASTAR_H

#include "config.h"

#define MAX_PATH_LEN 1024

typedef struct {
    int x, y, z;
} Node;

typedef struct {
    Node steps[MAX_PATH_LEN];
    int length;
    int valid;
    int survivor_id;
} Path;

typedef struct {
    Node pos;
    int id;
    int priority;
} Survivor;

Path astar(Node start, Node goal, const Config *cfg);
int is_valid(Node n, const Config *cfg);
int list_survivors(Survivor out[], int max, const Config *cfg);

#endif