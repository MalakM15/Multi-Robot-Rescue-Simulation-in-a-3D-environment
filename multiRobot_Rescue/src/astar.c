#include "all_headers.h"

// Helper function to check if a node is valid (within bounds and not an obstacle)
int is_valid(Node n, const Config *cfg) {
    if (!cfg || !building) return 0;
    
    // Check bounds
    if (n.x < 0 || n.x >= cfg->grid_x ||
        n.y < 0 || n.y >= cfg->grid_y ||
        n.z < 0 || n.z >= cfg->grid_z) {
        return 0;
    }
    
    // Check if cell is an obstacle
    if (building[n.z][n.y][n.x].obstacle) {
        return 0;
    }
    
    return 1;
}

static double heuristic(Node a, Node b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = a.z - b.z;
    return sqrt(dx*dx + dy*dy + dz*dz);
}

// Get neighbors of a node (6-directional: up, down, north, south, east, west)
static int get_neighbors(Node neighbors[], Node current, const Config *cfg) {
    int count = 0;
    Node offsets[] = {
        {0, 0, 1},   // up
        {0, 0, -1},  // down
        {0, 1, 0},   // north
        {0, -1, 0},  // south
        {1, 0, 0},   // east
        {-1, 0, 0}   // west
    };
    
    for (int i = 0; i < 6; i++) {
        Node neighbor = {
            current.x + offsets[i].x,
            current.y + offsets[i].y,
            current.z + offsets[i].z
        };
        
        if (is_valid(neighbor, cfg)) {
            neighbors[count++] = neighbor;
        }
    }
    
    return count;
}

// Check if two nodes are equal
static int node_equal(Node a, Node b) {
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

typedef struct {
    Node *nodes;
    double *g_scores;
    double *f_scores;
    Node *parents;
    int count;
    int capacity;
} NodeSet;

static NodeSet* nodeset_create(int capacity) {
    NodeSet *set = malloc(sizeof(NodeSet));
    if (!set) return NULL;
    
    set->nodes = malloc(capacity * sizeof(Node));
    set->g_scores = malloc(capacity * sizeof(double));
    set->f_scores = malloc(capacity * sizeof(double));
    set->parents = malloc(capacity * sizeof(Node));
    
    if (!set->nodes || !set->g_scores || !set->f_scores || !set->parents) {
        free(set->nodes);
        free(set->g_scores);
        free(set->f_scores);
        free(set->parents);
        free(set);
        return NULL;
    }
    
    set->count = 0;
    set->capacity = capacity;
    return set;
}

static void nodeset_free(NodeSet *set) {
    if (!set) return;
    free(set->nodes);
    free(set->g_scores);
    free(set->f_scores);
    free(set->parents);
    free(set);
}

static int nodeset_find(NodeSet *set, Node n) {
    for (int i = 0; i < set->count; i++) {
        if (node_equal(set->nodes[i], n)) {
            return i;
        }
    }
    return -1;
}

static int nodeset_add(NodeSet *set, Node n, double g, double f, Node parent) {
    if (set->count >= set->capacity) return 0;
    
    set->nodes[set->count] = n;
    set->g_scores[set->count] = g;
    set->f_scores[set->count] = f;
    set->parents[set->count] = parent;
    set->count++;
    return 1;
}

static void reconstruct_path(Node *path, int *path_len, Node current, 
                            NodeSet *closed_set, Node start) {
    *path_len = 0;
    Node temp_path[MAX_PATH_LEN];
    int temp_len = 0;
    
    // Build path backwards from goal to start
    Node node = current;
    temp_path[temp_len++] = node;
    
    while (!node_equal(node, start) && temp_len < MAX_PATH_LEN) {
        int idx = nodeset_find(closed_set, node);
        if (idx < 0) break;
        
        Node parent = closed_set->parents[idx];
        if (node_equal(node, parent)) break;
        
        node = parent;
        temp_path[temp_len++] = node;
    }
    
    // Reverse path to get start -> goal
    *path_len = temp_len;
    for (int i = 0; i < temp_len; i++) {
        path[i] = temp_path[temp_len - 1 - i];
    }
}

Path astar(Node start, Node goal, const Config *cfg) {
    Path result = {0};
    result.valid = 0;
    result.length = 0;
    result.survivor_id = -1;
    
    if (!cfg || !building) {
        return result;
    }
    
    // Check if start and goal are valid
    if (!is_valid(start, cfg) || !is_valid(goal, cfg)) {
        return result;
    }
    
    // If start equals goal, return trivial path
    if (node_equal(start, goal)) {
        result.steps[0] = start;
        result.length = 1;
        result.valid = 1;
        return result;
    }
    
    int max_nodes = cfg->grid_x * cfg->grid_y * cfg->grid_z;
    NodeSet *open_set = nodeset_create(max_nodes);
    NodeSet *closed_set = nodeset_create(max_nodes);
    
    if (!open_set || !closed_set) {
        if (open_set) nodeset_free(open_set);
        if (closed_set) nodeset_free(closed_set);
        return result;
    }
    
    double h_start = heuristic(start, goal);
    nodeset_add(open_set, start, 0.0, h_start, start);
    
    // A* main loop
    while (open_set->count > 0) {
        Node current;
        Node current_parent;
        double g_current;
        
        // Get the node with best f_score and its g_score
        int best_idx = 0;
        for (int i = 1; i < open_set->count; i++) {
            if (open_set->f_scores[i] < open_set->f_scores[best_idx]) {
                best_idx = i;
            }
        }
        
        current = open_set->nodes[best_idx];
        current_parent = open_set->parents[best_idx];
        g_current = open_set->g_scores[best_idx];
        
        open_set->nodes[best_idx] = open_set->nodes[open_set->count - 1];
        open_set->g_scores[best_idx] = open_set->g_scores[open_set->count - 1];
        open_set->f_scores[best_idx] = open_set->f_scores[open_set->count - 1];
        open_set->parents[best_idx] = open_set->parents[open_set->count - 1];
        open_set->count--;
        
        if (nodeset_find(closed_set, current) >= 0) {
            continue;
        }
        
        nodeset_add(closed_set, current, g_current, 
                   g_current + heuristic(current, goal), current_parent);
        
        // Check if we reached the goal
        if (node_equal(current, goal)) {
            reconstruct_path(result.steps, &result.length, current, 
                           closed_set, start);
            result.valid = 1;
            nodeset_free(open_set);
            nodeset_free(closed_set);
            return result;
        }
        
        // Explore neighbors
        Node neighbors[6];
        int neighbor_count = get_neighbors(neighbors, current, cfg);
        
        for (int i = 0; i < neighbor_count; i++) {
            Node neighbor = neighbors[i];
            
            if (nodeset_find(closed_set, neighbor) >= 0) {
                continue;
            }
            
            // Calculate cost (1.0 for movement, add risk penalty)
            double move_cost = 1.0;
            if (building[neighbor.z][neighbor.y][neighbor.x].risk > 0) {
                move_cost += building[neighbor.z][neighbor.y][neighbor.x].risk * 0.5;
            }
            
            double tentative_g = g_current + move_cost;
            
            // Check if neighbor is in open set
            int open_idx = nodeset_find(open_set, neighbor);
            if (open_idx >= 0) {
                // Already in open set, check if this path is better
                if (tentative_g < open_set->g_scores[open_idx]) {
                    open_set->g_scores[open_idx] = tentative_g;
                    open_set->f_scores[open_idx] = tentative_g + heuristic(neighbor, goal);
                    open_set->parents[open_idx] = current;
                }
            } else {
                // Add to open set
                double f_score = tentative_g + heuristic(neighbor, goal);
                nodeset_add(open_set, neighbor, tentative_g, f_score, current);
            }
        }
    }
    
    // No path found
    nodeset_free(open_set);
    nodeset_free(closed_set);
    return result;
}

int list_survivors(Survivor out[], int max, const Config *cfg) {
    if (!building || !cfg || !out || max <= 0) {
        return 0;
    }
    
    int count = 0;
    int survivor_id = 0;
    
    for (int z = 0; z < cfg->grid_z && count < max; z++) {
        for (int y = 0; y < cfg->grid_y && count < max; y++) {
            for (int x = 0; x < cfg->grid_x && count < max; x++) {
                if (building[z][y][x].survivor) {
                    out[count].pos.x = x;
                    out[count].pos.y = y;
                    out[count].pos.z = z;
                    out[count].id = survivor_id++;
                    out[count].priority = 3 - building[z][y][x].risk;
                    count++;
                }
            }
        }
    }
    
    return count;
}

