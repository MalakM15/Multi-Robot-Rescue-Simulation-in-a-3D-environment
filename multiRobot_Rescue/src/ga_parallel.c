#include "all_headers.h"

// Forward declarations
void cleanup_shared_memory(void);
void shutdown_process_pool(void);

// SIGCHLD handler to reap zombie
static void sigchld_handler(int sig) {
    (void)sig;
    pid_t pid;
    int status;
    // Reap all zombie children (use WNOHANG to avoid blocking)
    while ((pid = waitpid(-1, &status, WNOHANG)) > 0) {
   
    }
}

static void atexit_cleanup(void) {
    shutdown_process_pool();
}

static void signal_handler(int sig) {
    (void)sig;
    sigchld_handler(sig);
    shutdown_process_pool();
    _exit(1);
}

// Register signal handlers for cleanup
static void register_signal_handlers(void) {
    // Install SIGCHLD handler to reap zombies immediately
    signal(SIGCHLD, sigchld_handler);
    signal(SIGINT, signal_handler);    // Ctrl+C
    signal(SIGTERM, signal_handler);   // Termination request
    signal(SIGSEGV, signal_handler);   // Segmentation fault
    signal(SIGABRT, signal_handler);   // Abort
    signal(SIGTSTP, signal_handler);   // Ctrl+Z
    atexit(atexit_cleanup);            // Normal exit
}

// Shared memory structure
typedef struct {
    int pop_size;
    int robot_count;
    int survivor_count;
    int num_workers;
    int next_chromosome;  
    int shutdown_flag;   
    int work_ready;       
    int max_survivors_per_robot;  
    size_t shm_data_size; 
    size_t shm_chromosomes_size;  
    double *fitness_results;  
    Config config;
    Node *robot_starts;  
    Survivor *survivors; 
} SharedData;

typedef struct {
    RobotMission *missions; 
    int *survivor_sequences; 
    int *survivor_counts;  
    Node *robot_positions; 
} SharedChromosomes;

#define SHM_NAME "/ga_shared_mem"
#define SHM_CHROMOSOMES_NAME "/ga_chromosomes_mem"
#define SEM_WORK_NAME "/ga_work_sem"
#define SEM_RESULT_NAME "/ga_result_sem"
#define SEM_MUTEX_NAME "/ga_mutex_sem"

SharedData *shared_data = NULL; 
SharedChromosomes *shared_chromosomes = NULL; 
sem_t *sem_work = NULL; 
sem_t *sem_result = NULL; 
sem_t *sem_mutex = NULL; 
static int shm_fd = -1;
static int shm_chromosomes_fd = -1;
static pid_t *worker_pids = NULL; 
static int num_worker_processes = 0; 
static int pool_initialized = 0;  

int init_shared_memory(int pop_size, int robot_count, int survivor_count, int num_workers, int max_survivors_per_robot) {
   
    if (pop_size <= 0) {
        fprintf(stderr, "Warning: Invalid pop_size (%d), using default 1000\n", pop_size);
        pop_size = 1000;
    }
    if (robot_count <= 0) {
        fprintf(stderr, "Warning: Invalid robot_count (%d), using default 100\n", robot_count);
        robot_count = 100;
    }
    if (survivor_count <= 0) {
        fprintf(stderr, "Warning: Invalid survivor_count (%d), using default 10000\n", survivor_count);
        survivor_count = 10000;
    }
    if (max_survivors_per_robot <= 0) {
        fprintf(stderr, "Warning: Invalid max_survivors_per_robot (%d), using default 100\n", max_survivors_per_robot);
        max_survivors_per_robot = 100;
    }

    shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
        perror("shm_open (main)");
        return -1;
    }
    
    // Calculate total size needed
    size_t base_size = sizeof(SharedData);
    size_t fitness_size = pop_size * sizeof(double);
    size_t robot_starts_size = robot_count * sizeof(Node);
    size_t survivors_size = survivor_count * sizeof(Survivor);
    size_t shm_size = base_size + fitness_size + robot_starts_size + survivors_size;
    
    if (ftruncate(shm_fd, shm_size) == -1) {
        perror("ftruncate (main)");
        close(shm_fd);
        shm_unlink(SHM_NAME);
        return -1;
    }
    
    shared_data = mmap(NULL, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shared_data == MAP_FAILED) {
        perror("mmap (main)");
        close(shm_fd);
        shm_unlink(SHM_NAME);
        return -1;
    }
    

    memset(shared_data, 0, base_size);
    
    // Set up array pointers within shared memory
    char *base_ptr = (char *)shared_data;
    shared_data->fitness_results = (double *)(base_ptr + base_size);
    shared_data->robot_starts = (Node *)(base_ptr + base_size + fitness_size);
    shared_data->survivors = (Survivor *)(base_ptr + base_size + fitness_size + robot_starts_size);
    
    // Set the data fields
    shared_data->pop_size = pop_size;
    shared_data->robot_count = robot_count;
    shared_data->survivor_count = survivor_count;
    shared_data->num_workers = num_workers;
    shared_data->max_survivors_per_robot = max_survivors_per_robot;
    shared_data->shm_data_size = shm_size;
    shared_data->next_chromosome = 0;
    shared_data->shutdown_flag = 0;
    shared_data->work_ready = 0;
    
    shm_chromosomes_fd = shm_open(SHM_CHROMOSOMES_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_chromosomes_fd == -1) {
        perror("shm_open (chromosomes)");
        munmap(shared_data, shm_size);
        close(shm_fd);
        shm_unlink(SHM_NAME);
        return -1;
    }
    
    // Calculate total size needed for chromosomes
    size_t chromosomes_base_size = sizeof(SharedChromosomes);
    size_t missions_size = pop_size * robot_count * sizeof(RobotMission);
    size_t survivor_sequences_size = pop_size * robot_count * max_survivors_per_robot * sizeof(int);
    size_t survivor_counts_size = pop_size * robot_count * sizeof(int);
    size_t robot_positions_size = pop_size * robot_count * sizeof(Node);
    size_t shm_chromosomes_size = chromosomes_base_size + missions_size + survivor_sequences_size + 
                                   survivor_counts_size + robot_positions_size;
    
    if (ftruncate(shm_chromosomes_fd, shm_chromosomes_size) == -1) {
        perror("ftruncate (chromosomes)");
        close(shm_chromosomes_fd);
        munmap(shared_data, shm_size);
        close(shm_fd);
        shm_unlink(SHM_NAME);
        shm_unlink(SHM_CHROMOSOMES_NAME);
        return -1;
    }
    
    shared_chromosomes = mmap(NULL, shm_chromosomes_size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_chromosomes_fd, 0);
    if (shared_chromosomes == MAP_FAILED) {
        perror("mmap (chromosomes)");
        close(shm_chromosomes_fd);
        munmap(shared_data, shm_size);
        close(shm_fd);
        shm_unlink(SHM_NAME);
        shm_unlink(SHM_CHROMOSOMES_NAME);
        return -1;
    }
    
    memset(shared_chromosomes, 0, chromosomes_base_size);
    
    // Set up array pointers within shared memory for chromosomes
    char *chromosomes_base_ptr = (char *)shared_chromosomes;
    shared_chromosomes->missions = (RobotMission *)(chromosomes_base_ptr + chromosomes_base_size);
    shared_chromosomes->survivor_sequences = (int *)(chromosomes_base_ptr + chromosomes_base_size + missions_size);
    shared_chromosomes->survivor_counts = (int *)(chromosomes_base_ptr + chromosomes_base_size + missions_size + survivor_sequences_size);
    shared_chromosomes->robot_positions = (Node *)(chromosomes_base_ptr + chromosomes_base_size + missions_size + 
                                                    survivor_sequences_size + survivor_counts_size);
    
    // Store chromosomes size in shared_data
    shared_data->shm_chromosomes_size = shm_chromosomes_size;
    
    // Initialize arrays to zero
    memset(shared_data->fitness_results, 0, fitness_size);
    memset(shared_data->robot_starts, 0, robot_starts_size);
    memset(shared_data->survivors, 0, survivors_size);
    memset(shared_chromosomes->missions, 0, missions_size);
    memset(shared_chromosomes->survivor_sequences, 0, survivor_sequences_size);
    memset(shared_chromosomes->survivor_counts, 0, survivor_counts_size);
    memset(shared_chromosomes->robot_positions, 0, robot_positions_size);
    
    sem_work = sem_open(SEM_WORK_NAME, O_CREAT | O_RDWR, 0666, 0);
    sem_result = sem_open(SEM_RESULT_NAME, O_CREAT | O_RDWR, 0666, 0);
    sem_mutex = sem_open(SEM_MUTEX_NAME, O_CREAT | O_RDWR, 0666, 1);
    
    if (sem_work == SEM_FAILED || sem_result == SEM_FAILED || sem_mutex == SEM_FAILED) {
        perror("sem_open");
        cleanup_shared_memory();
        return -1;
    }
    
    return 0;
}


void cleanup_shared_memory(void) {
    if (sem_work != NULL && sem_work != SEM_FAILED) {
        sem_close(sem_work);
        sem_unlink(SEM_WORK_NAME);
        sem_work = NULL;
    }
    if (sem_result != NULL && sem_result != SEM_FAILED) {
        sem_close(sem_result);
        sem_unlink(SEM_RESULT_NAME);
        sem_result = NULL;
    }
    if (sem_mutex != NULL && sem_mutex != SEM_FAILED) {
        sem_close(sem_mutex);
        sem_unlink(SEM_MUTEX_NAME);
        sem_mutex = NULL;
    }
    
    if (shared_chromosomes != NULL && shared_chromosomes != MAP_FAILED) {
        size_t chromosomes_size = (shared_data && shared_data->shm_chromosomes_size > 0) ? 
                                   shared_data->shm_chromosomes_size : sizeof(SharedChromosomes);
        munmap(shared_chromosomes, chromosomes_size);
        shared_chromosomes = NULL;
    }
    if (shm_chromosomes_fd != -1) {
        close(shm_chromosomes_fd);
        shm_unlink(SHM_CHROMOSOMES_NAME);
        shm_chromosomes_fd = -1;
    }
    
    if (shared_data != NULL && shared_data != MAP_FAILED) {
        size_t data_size = shared_data->shm_data_size > 0 ? shared_data->shm_data_size : sizeof(SharedData);
        munmap(shared_data, data_size);
        shared_data = NULL;
    }
    if (shm_fd != -1) {
        close(shm_fd);
        shm_unlink(SHM_NAME);
        shm_fd = -1;
    }
    
    if (sem_work != NULL && sem_work != SEM_FAILED) {
        sem_close(sem_work);
        sem_unlink(SEM_WORK_NAME);
        sem_work = NULL;
    }
    if (sem_result != NULL && sem_result != SEM_FAILED) {
        sem_close(sem_result);
        sem_unlink(SEM_RESULT_NAME);
        sem_result = NULL;
    }
    if (sem_mutex != NULL && sem_mutex != SEM_FAILED) {
        sem_close(sem_mutex);
        sem_unlink(SEM_MUTEX_NAME);
        sem_mutex = NULL;
    }
}

void copy_population_to_shared(Chromosome pop[], int pop_size, int robot_count,
                               const Node robot_starts[], const Survivor survivors[]) {
    if (!shared_data || !shared_chromosomes) return;
    
    // Copy configuration data
    memcpy(shared_data->robot_starts, robot_starts, robot_count * sizeof(Node));
    memcpy(shared_data->survivors, survivors, shared_data->survivor_count * sizeof(Survivor));
    
    // Copy chromosome data
    int max_surv = shared_data->max_survivors_per_robot;
    for (int i = 0; i < pop_size; i++) {
        for (int r = 0; r < robot_count; r++) {
            int idx = i * robot_count + r;
            shared_chromosomes->robot_positions[idx] = pop[i].missions[r].robot_pos;
            shared_chromosomes->survivor_counts[idx] = pop[i].missions[r].survivor_count;
            
            // Copy survivor sequence
            int seq_base = i * robot_count * max_surv + r * max_surv;
            int actual_count = pop[i].missions[r].survivor_count;
            if (actual_count > max_surv) actual_count = max_surv;
            for (int s = 0; s < actual_count; s++) {
                shared_chromosomes->survivor_sequences[seq_base + s] = pop[i].missions[r].survivor_sequence[s];
            }
        }
    }
    
    shared_data->next_chromosome = 0;
}

void copy_fitness_from_shared(Chromosome pop[], int pop_size) {
    if (!shared_data) return;
    
    for (int i = 0; i < pop_size; i++) {
        pop[i].fitness = shared_data->fitness_results[i];
    }
}

void worker_process(void) {
    int shm_fd = shm_open(SHM_NAME, O_RDWR, 0666);
    if (shm_fd == -1) {
        perror("worker: shm_open");
        exit(1);
    }
    
    SharedData *temp_shared = mmap(NULL, sizeof(SharedData), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (temp_shared == MAP_FAILED) {
        perror("worker: mmap (temp)");
        close(shm_fd);
        exit(1);
    }
    
    // Get the actual size
    size_t shm_data_size = temp_shared->shm_data_size;
    if (shm_data_size == 0 || shm_data_size < sizeof(SharedData)) {
        // Fallback: use a reasonable default size
        shm_data_size = sizeof(SharedData) + 1000 * sizeof(double) + 100 * sizeof(Node) + 10000 * sizeof(Survivor);
    }
    munmap(temp_shared, sizeof(SharedData));
    
    SharedData *local_shared = mmap(NULL, shm_data_size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (local_shared == MAP_FAILED) {
        perror("worker: mmap");
        close(shm_fd);
        exit(1);
    }
    
    // Calculate array pointers locally
    int pop_size_val = local_shared->pop_size > 0 ? local_shared->pop_size : 1000;
    int robot_count_val = local_shared->robot_count > 0 ? local_shared->robot_count : 100;
    size_t base_size = sizeof(SharedData);
    char *base_ptr = (char *)local_shared;
    if (!base_ptr) {
        fprintf(stderr, "Worker: Invalid base pointer\n");
        exit(1);
    }
    
    double *local_fitness_results = (double *)(base_ptr + base_size);
    size_t fitness_size = pop_size_val * sizeof(double);
    size_t robot_starts_size = robot_count_val * sizeof(Node);
    Survivor *local_survivors = (Survivor *)(base_ptr + base_size + fitness_size + robot_starts_size);
    
    if ((char *)local_fitness_results >= (char *)local_shared + shm_data_size ||
        (char *)local_survivors >= (char *)local_shared + shm_data_size) {
        fprintf(stderr, "Worker: Calculated pointers out of bounds\n");
        exit(1);
    }
    
    int shm_chromosomes_fd = shm_open(SHM_CHROMOSOMES_NAME, O_RDWR, 0666);
    if (shm_chromosomes_fd == -1) {
        perror("worker: shm_open chromosomes");
        exit(1);
    }
    
    // Get chromosomes size from shared data
    size_t shm_chromosomes_size = local_shared->shm_chromosomes_size;
    if (shm_chromosomes_size == 0) {
        int pop_size = local_shared->pop_size > 0 ? local_shared->pop_size : 1000;
        int robot_count = local_shared->robot_count > 0 ? local_shared->robot_count : 100;
        int max_surv = local_shared->max_survivors_per_robot > 0 ? local_shared->max_survivors_per_robot : 100;
        size_t chromosomes_base_size = sizeof(SharedChromosomes);
        size_t missions_size = pop_size * robot_count * sizeof(RobotMission);
        size_t survivor_sequences_size = pop_size * robot_count * max_surv * sizeof(int);
        size_t survivor_counts_size = pop_size * robot_count * sizeof(int);
        size_t robot_positions_size = pop_size * robot_count * sizeof(Node);
        shm_chromosomes_size = chromosomes_base_size + missions_size + survivor_sequences_size + 
                               survivor_counts_size + robot_positions_size;
    }
    
    SharedChromosomes *local_chromosomes = mmap(NULL, shm_chromosomes_size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_chromosomes_fd, 0);
    if (local_chromosomes == MAP_FAILED) {
        perror("worker: mmap chromosomes");
        close(shm_chromosomes_fd);
        exit(1);
    }
    
    // Calculate array pointers locally for chromosomes
    size_t chromosomes_base_size = sizeof(SharedChromosomes);
    char *chromosomes_base_ptr = (char *)local_chromosomes;
    int pop_size = local_shared->pop_size > 0 ? local_shared->pop_size : 1000;
    int robot_count = local_shared->robot_count > 0 ? local_shared->robot_count : 100;
    int max_surv = local_shared->max_survivors_per_robot > 0 ? local_shared->max_survivors_per_robot : 100;
    
    // Calculate local pointers
    size_t missions_size = pop_size * robot_count * sizeof(RobotMission);
    size_t survivor_sequences_size = pop_size * robot_count * max_surv * sizeof(int);
    size_t survivor_counts_size = pop_size * robot_count * sizeof(int);
    
    int *local_survivor_sequences = (int *)(chromosomes_base_ptr + chromosomes_base_size + missions_size);
    int *local_survivor_counts = (int *)(chromosomes_base_ptr + chromosomes_base_size + missions_size + survivor_sequences_size);
    Node *local_robot_positions = (Node *)(chromosomes_base_ptr + chromosomes_base_size + missions_size + 
                                           survivor_sequences_size + survivor_counts_size);
    
    sem_t *sem_work_local = sem_open(SEM_WORK_NAME, 0);
    sem_t *sem_result_local = sem_open(SEM_RESULT_NAME, 0);
    sem_t *sem_mutex_local = sem_open(SEM_MUTEX_NAME, 0);
    
    if (sem_work_local == SEM_FAILED || sem_result_local == SEM_FAILED || sem_mutex_local == SEM_FAILED) {
        perror("worker: sem_open");
        exit(1);
    }
    
    while (1) {
        if (local_shared->shutdown_flag) {
            break;
        }
        
        // Wait for work
        if (sem_wait(sem_work_local) != 0) {
            perror("worker: sem_wait (work)");
            break;
        }
        
        if (local_shared->shutdown_flag) {
            sem_post(sem_result_local); 
            break;
        }
        
        // Get next chromosome index
        if (sem_wait(sem_mutex_local) != 0) {
            perror("worker: sem_wait (mutex)");
            sem_post(sem_result_local);
            break;
        }
        int chrom_idx = local_shared->next_chromosome++;
        int pop_size = local_shared->pop_size;
        sem_post(sem_mutex_local);
        
        if (chrom_idx >= pop_size) {
            sem_post(sem_result_local);
            continue;
        }
        
        // Validate bounds
        if (chrom_idx < 0 || chrom_idx >= pop_size) {
            fprintf(stderr, "Worker: Invalid chromosome index %d (pop_size=%d)\n", chrom_idx, pop_size);
            sem_post(sem_result_local);
            continue;
        }
        
        Chromosome chrom;
        int robot_count = local_shared->robot_count;
        if (robot_count <= 0) {
            fprintf(stderr, "Worker: Invalid robot_count %d\n", robot_count);
            sem_post(sem_result_local);
            continue;
        }
        
        chrom.missions = malloc(robot_count * sizeof(RobotMission));
        if (!chrom.missions) {
            fprintf(stderr, "Worker: Failed to allocate memory\n");
            exit(1);
        }
        
        int max_surv = local_shared->max_survivors_per_robot;
        if (max_surv <= 0) max_surv = 100;  // Safety fallback
        
        for (int r = 0; r < robot_count; r++) {
            // Use flat indexing with local pointers
            int pos_idx = chrom_idx * robot_count + r;
            if (pos_idx < 0) {
                fprintf(stderr, "Worker: Invalid position index %d\n", pos_idx);
                free(chrom.missions);
                sem_post(sem_result_local);
                continue;
            }
            chrom.missions[r].robot_pos = local_robot_positions[pos_idx];
            chrom.missions[r].survivor_count = local_survivor_counts[pos_idx];

            chrom.missions[r].survivor_sequence = malloc(max_surv * sizeof(int));
            if (!chrom.missions[r].survivor_sequence) {
                // Free previously allocated arrays
                for (int rr = 0; rr < r; rr++) {
                    free(chrom.missions[rr].survivor_sequence);
                }
                free(chrom.missions);
                fprintf(stderr, "Worker: Failed to allocate survivor_sequence memory\n");
                exit(1);
            }
            int seq_base = chrom_idx * robot_count * max_surv + r * max_surv;
            int actual_count = chrom.missions[r].survivor_count;
            if (actual_count > max_surv) actual_count = max_surv;
            for (int s = 0; s < actual_count; s++) {
                chrom.missions[r].survivor_sequence[s] = local_survivor_sequences[seq_base + s];
            }
        }
        
        // The building grid is a global variable accessible in forked processes
        // Since we fork after allocating the grid, child processes inherit the pointer
        double fitness = fitness_chromosome(&chrom, local_shared->robot_count, &local_shared->config);
        
        // Store result using local pointer
        local_fitness_results[chrom_idx] = fitness;
        
        // Free allocated memory
        for (int r = 0; r < local_shared->robot_count; r++) {
            free(chrom.missions[r].survivor_sequence);
        }
        free(chrom.missions);
        
        // Signal result ready
        if (sem_post(sem_result_local) != 0) {
            perror("worker: sem_post (result)");
            break;
        }
    }
    
    // Cleanup worker resources
    munmap(local_shared, local_shared->shm_data_size);
    close(shm_fd);
    munmap(local_chromosomes, local_shared->shm_chromosomes_size);
    close(shm_chromosomes_fd);
    sem_close(sem_work_local);
    sem_close(sem_result_local);
    sem_close(sem_mutex_local);
    exit(0);
}

// Initialize process pool
int init_process_pool(int max_pop_size, int robot_count, int max_survivor_count, int pool_size, int max_survivors_per_robot) {
    if (pool_initialized) {
        return 0;  
    }
    
    num_worker_processes = pool_size > 0 ? pool_size : 1;  // Ensure at least one worker
    
    // Check if building grid is allocated
    extern Cell ***building;
    if (building == NULL) {
        return -1;  // Cannot use process pool without building grid
    }
    
    // Initialize shared memory
    if (init_shared_memory(max_pop_size, robot_count, max_survivor_count, num_worker_processes, max_survivors_per_robot) != 0) {
        return -1;
    }
    
    // Allocate array for worker PIDs
    worker_pids = malloc(num_worker_processes * sizeof(pid_t));
    if (!worker_pids) {
        cleanup_shared_memory();
        return -1;
    }
    
    // Create worker processes
    for (int i = 0; i < num_worker_processes; i++) {
        pid_t pid = fork();
        if (pid == 0) {
            // Child process, worker
            worker_process();
            exit(0);
        } else if (pid > 0) {
            // Parent process, store worker PID
            worker_pids[i] = pid;
        } else {
            perror("fork");
            for (int j = 0; j < i; j++) {
                kill(worker_pids[j], SIGTERM);
            }
            free(worker_pids);
            worker_pids = NULL;
            cleanup_shared_memory();
            return -1;
        }
    }
    
    pool_initialized = 1;
    
    // Register signal handlers to ensure cleanup on interruption
    register_signal_handlers();
    
    return 0;
}

// Shutdown process pool
void shutdown_process_pool(void) {
    if (!pool_initialized) {
        return;
    }
    
    pool_initialized = 0;
    
    // Signal workers to shutdown
    if (shared_data) {
        shared_data->shutdown_flag = 1;
        
        // Wake up all workers (to exit)
        if (sem_work != NULL && sem_work != SEM_FAILED) {
            for (int i = 0; i < num_worker_processes; i++) {
                sem_post(sem_work);
            }
        }
    }
    
    // Force kill all workers immediately
    if (worker_pids) {
        for (int i = 0; i < num_worker_processes; i++) {
            if (worker_pids[i] > 0) {
                // Send SIGTERM first for graceful shutdown
                kill(worker_pids[i], SIGTERM);
            }
        }
        
        // Wait a brief moment for graceful shutdown
        usleep(100000);  // 100ms
        
        //  force kill any that didn't exit gracefully
        for (int i = 0; i < num_worker_processes; i++) {
            if (worker_pids[i] > 0) {
                kill(worker_pids[i], SIGKILL);
            }
        }
        
        // Use a timeout loop to ensure we wait for all children
        int wait_attempts = 0;
        const int max_wait_attempts = 100;  // 10 seconds total (100 * 100ms)
        
        while (wait_attempts < max_wait_attempts) {
            int all_reaped = 1;
            
            for (int i = 0; i < num_worker_processes; i++) {
                if (worker_pids[i] > 0) {
                    int status;
                    pid_t waited = waitpid(worker_pids[i], &status, WNOHANG);
                    
                    if (waited == worker_pids[i]) {
                        // This child is killed
                        worker_pids[i] = 0;
                    } else if (waited == 0) {
                        // Child still alive
                        all_reaped = 0;
                    } else if (waited == -1 && errno == ECHILD) {
                        // No such child 
                        worker_pids[i] = 0;
                    } else {
                        if (errno != EINTR) {
                            all_reaped = 0;
                        }
                    }
                }
            }
            
            if (all_reaped) {
                break;  // All children killed
            }
            
            // Wait a bit before trying again
            usleep(100000);  // 100ms
            wait_attempts++;
        }
        
        // Force kill any remaining children that didn't exit
        for (int i = 0; i < num_worker_processes; i++) {
            if (worker_pids[i] > 0) {
                kill(worker_pids[i], SIGKILL);
                // Wait for it one more time
                int status;
                pid_t waited;
                do {
                    waited = waitpid(worker_pids[i], &status, 0);
                } while (waited == -1 && errno == EINTR);
                worker_pids[i] = 0;
            }
        }
        
        free(worker_pids);
        worker_pids = NULL;
    }
    
    // cleanup kill any zombie children
    pid_t pid;
    int status;
    while ((pid = waitpid(-1, &status, WNOHANG)) > 0) {
    }
    
    cleanup_shared_memory();
    
    num_worker_processes = 0;
}

// Parallel fitness computation using multiprocessing with process pool
void compute_fitness_parallel_mp(Chromosome pop[], int pop_size, int robot_count, const Config *cfg,
                                  const Node robot_starts[], const Survivor survivors[], int survivor_count) {
    // Initialize process pool if not already done
    static int pool_init_attempted = 0;
    
    if (!pool_init_attempted) {
        int pool_size = (cfg && cfg->pool_size > 0) ? cfg->pool_size : 1;
        int max_survivors_per_robot = (cfg && cfg->max_survivors_per_robot > 0) ? cfg->max_survivors_per_robot : 100;
        // Calculate max survivors: max_survivors_per_robot * population_size
        int calculated_max_survivors = max_survivors_per_robot * pop_size;
        if (init_process_pool(pop_size, robot_count, calculated_max_survivors, pool_size, max_survivors_per_robot) != 0) {
            pool_init_attempted = 1;
            for (int i = 0; i < pop_size; i++) {
                pop[i].fitness = fitness_chromosome(&pop[i], robot_count, cfg);
            }
            return;
        }
        pool_init_attempted = 1;
    }
    
    // Check if pool is initialized
    if (!pool_initialized || !shared_data) {
        // Fallback to sequential
        for (int i = 0; i < pop_size; i++) {
            pop[i].fitness = fitness_chromosome(&pop[i], robot_count, cfg);
        }
        return;
    }
    
    // Update shared memory with new work
    shared_data->pop_size = pop_size;
    shared_data->robot_count = robot_count;
    int calculated_max_survivors = cfg->max_survivors_per_robot * pop_size;
    shared_data->survivor_count = (survivor_count < calculated_max_survivors) ? survivor_count : calculated_max_survivors;
    shared_data->next_chromosome = 0;
    shared_data->shutdown_flag = 0;  
    // Copy configuration to shared memory
    memcpy(&shared_data->config, cfg, sizeof(Config));
    // Store max_survivors_per_robot for worker processes
    shared_data->max_survivors_per_robot = cfg->max_survivors_per_robot;
    
    // Copy population to shared memory
    copy_population_to_shared(pop, pop_size, robot_count, robot_starts, survivors);
    
    // Signal workers to start
    for (int i = 0; i < pop_size; i++) {
        if (sem_post(sem_work) != 0) {
            perror("sem_post (work)");
            // Fallback to sequential
            for (int i = 0; i < pop_size; i++) {
                pop[i].fitness = fitness_chromosome(&pop[i], robot_count, cfg);
            }
            return;
        }
    }
    
    // Wait for all results
    for (int i = 0; i < pop_size; i++) {
        if (sem_wait(sem_result) != 0) {
            perror("sem_wait (result)");
            break;
        }
    }
    
    // Copy results back
    copy_fitness_from_shared(pop, pop_size);
    
}
