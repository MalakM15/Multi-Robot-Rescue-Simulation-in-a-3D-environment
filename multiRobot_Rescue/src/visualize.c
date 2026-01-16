#include "all_headers.h"

// Camera settings
static float camera_angle_x = 45.0f;
static float camera_angle_y = 45.0f;
static float camera_distance = 20.0f;
static int mouse_x = 0;
static int mouse_y = 0;
static int mouse_button = 0;

// Visualization data
static Config vis_config;
static Path *vis_paths = NULL;
static Survivor *vis_survivors = NULL;
static int vis_robot_count = 0;
static int vis_survivor_count = 0;
static Node *vis_robot_starts = NULL;

static void robot_color_for_index(int idx, int total, float *r, float *g, float *b) {
    (void)total; 
    
    static const float colors[][3] = {
        {1.0f, 0.2f, 0.2f},   // Bright Red
        {0.2f, 1.0f, 0.2f},   // Bright Green  
        {0.3f, 0.6f, 1.0f},   // Bright Blue
        {1.0f, 1.0f, 0.2f},   // Bright Yellow
        {1.0f, 0.2f, 1.0f},   // Bright Magenta
        {0.2f, 1.0f, 1.0f},   // Bright Cyan
        {1.0f, 0.6f, 0.2f},   // Bright Orange
        {0.6f, 1.0f, 0.6f},   // Light Green
        {1.0f, 0.6f, 0.8f},   // Pink
        {0.8f, 0.8f, 1.0f},   // Light Blue
        {1.0f, 1.0f, 0.6f},   // Light Yellow
        {0.6f, 1.0f, 1.0f},   // Light Cyan
        {1.0f, 0.8f, 0.4f},   // Gold
        {0.8f, 0.4f, 1.0f},   // Purple
        {0.4f, 1.0f, 0.8f},   // Turquoise
        {1.0f, 0.4f, 0.6f},   // Coral
    };
    
    int num_colors = sizeof(colors) / sizeof(colors[0]);
    int color_idx = idx % num_colors;
    
    *r = colors[color_idx][0];
    *g = colors[color_idx][1];
    *b = colors[color_idx][2];
}

// Color definitions
#define COLOR_FREE     0.6f, 0.6f, 0.6f   // Light Gray = Free/empty grid cells (traversable)
#define COLOR_OBSTACLE 0.7f, 0.35f, 0.2f  // Bright Brown/Orange = Obstacles/debris (blocked)
#define COLOR_SURVIVOR 1.0f, 1.0f, 0.0f   // Bright Yellow = Survivors

void draw_cube(float x, float y, float z, float size, float r, float g, float b) {
    glPushMatrix();
    glTranslatef(x, y, z);
    
    // Disable lighting so colors stay constant regardless of rotation
    glDisable(GL_LIGHTING);
    glColor3f(r, g, b);
    
    // Draw cube faces
    glBegin(GL_QUADS);
    
    // Front face
    glVertex3f(-size/2, -size/2, size/2);
    glVertex3f(size/2, -size/2, size/2);
    glVertex3f(size/2, size/2, size/2);
    glVertex3f(-size/2, size/2, size/2);
    
    // Back face
    glVertex3f(-size/2, -size/2, -size/2);
    glVertex3f(-size/2, size/2, -size/2);
    glVertex3f(size/2, size/2, -size/2);
    glVertex3f(size/2, -size/2, -size/2);
    
    // Top face
    glVertex3f(-size/2, size/2, -size/2);
    glVertex3f(-size/2, size/2, size/2);
    glVertex3f(size/2, size/2, size/2);
    glVertex3f(size/2, size/2, -size/2);
    
    // Bottom face
    glVertex3f(-size/2, -size/2, -size/2);
    glVertex3f(size/2, -size/2, -size/2);
    glVertex3f(size/2, -size/2, size/2);
    glVertex3f(-size/2, -size/2, size/2);
    
    // Right face
    glVertex3f(size/2, -size/2, -size/2);
    glVertex3f(size/2, size/2, -size/2);
    glVertex3f(size/2, size/2, size/2);
    glVertex3f(size/2, -size/2, size/2);
    
    // Left face
    glVertex3f(-size/2, -size/2, -size/2);
    glVertex3f(-size/2, -size/2, size/2);
    glVertex3f(-size/2, size/2, size/2);
    glVertex3f(-size/2, size/2, -size/2);
    
    glEnd();
    
    // Draw wireframe edges 
    glColor3f(0.6f, 0.6f, 0.6f);
    glLineWidth(1.5f);
    glBegin(GL_LINE_LOOP);
    glVertex3f(-size/2, -size/2, size/2);
    glVertex3f(size/2, -size/2, size/2);
    glVertex3f(size/2, size/2, size/2);
    glVertex3f(-size/2, size/2, size/2);
    glEnd();
    
    glEnable(GL_LIGHTING);
    
    glPopMatrix();
}

void draw_grid(const Config *cfg) {
    if (!cfg || !building) return;
    
    float cell_size = 0.8f;
    float spacing = 1.0f;
    
    // Draw grid cells
    for (int z = 0; z < cfg->grid_z; z++) {
        for (int y = 0; y < cfg->grid_y; y++) {
            for (int x = 0; x < cfg->grid_x; x++) {
                float px = x * spacing - (cfg->grid_x * spacing) / 2.0f;
                float py = y * spacing - (cfg->grid_y * spacing) / 2.0f;
                float pz = z * spacing;
                
                if (building[z][y][x].obstacle) {
                    // Dark gray for obstacles/debris 
                    draw_cube(px, py, pz, cell_size, 0.2f, 0.2f, 0.2f);
                } else {
                    // White for free/empty cells 
                    draw_cube(px, py, pz, cell_size * 0.5f, 1.0f, 1.0f, 1.0f);
                }
            }
        }
    }
}

void draw_human(float x, float y, float z, float scale) {
    glPushMatrix();
    glTranslatef(x, y, z);
    glScalef(scale, scale, scale);
    glColor3f(COLOR_SURVIVOR);
    
    // Draw head (sphere/cube)
    glPushMatrix();
    glTranslatef(0.0f, 0.4f, 0.0f);
    glutSolidSphere(0.15f, 10, 10);
    glPopMatrix();
    
    // Draw body (torso)
    glPushMatrix();
    glTranslatef(0.0f, 0.1f, 0.0f);
    glScalef(0.2f, 0.3f, 0.1f);
    glutSolidCube(1.0f);
    glPopMatrix();
    
    // Draw left arm
    glPushMatrix();
    glTranslatef(-0.2f, 0.15f, 0.0f);
    glRotatef(45.0f, 0.0f, 0.0f, 1.0f);
    glScalef(0.08f, 0.25f, 0.08f);
    glutSolidCube(1.0f);
    glPopMatrix();
    
    // Draw right arm
    glPushMatrix();
    glTranslatef(0.2f, 0.15f, 0.0f);
    glRotatef(-45.0f, 0.0f, 0.0f, 1.0f);
    glScalef(0.08f, 0.25f, 0.08f);
    glutSolidCube(1.0f);
    glPopMatrix();
    
    // Draw left leg
    glPushMatrix();
    glTranslatef(-0.1f, -0.2f, 0.0f);
    glScalef(0.1f, 0.3f, 0.1f);
    glutSolidCube(1.0f);
    glPopMatrix();
    
    // Draw right leg
    glPushMatrix();
    glTranslatef(0.1f, -0.2f, 0.0f);
    glScalef(0.1f, 0.3f, 0.1f);
    glutSolidCube(1.0f);
    glPopMatrix();
    
    glPopMatrix();
}

void draw_survivors(const Survivor *survivors, int count, const Config *cfg) {
    if (!survivors || !cfg) return;
    
    float spacing = 1.0f;
    float scale = 0.4f;
    
    for (int i = 0; i < count; i++) {
        float px = survivors[i].pos.x * spacing - (cfg->grid_x * spacing) / 2.0f;
        float py = survivors[i].pos.y * spacing - (cfg->grid_y * spacing) / 2.0f;
        float pz = survivors[i].pos.z * spacing;
        
        // Draw survivor as human figure
        draw_human(px, py, pz, scale);
    }
}

void draw_car(float x, float y, float z, float scale, float r, float g, float b) {
    glPushMatrix();
    glTranslatef(x, y, z);
    glScalef(scale, scale, scale);
    glColor3f(r, g, b);
    
    // Car body (main rectangle)
    glPushMatrix();
    glTranslatef(0.0f, 0.1f, 0.0f);
    glScalef(0.4f, 0.15f, 0.2f);
    glutSolidCube(1.0f);
    glPopMatrix();
    
    // Car roof (smaller rectangle on top)
    glPushMatrix();
    glTranslatef(0.0f, 0.25f, 0.0f);
    glScalef(0.3f, 0.1f, 0.18f);
    glutSolidCube(1.0f);
    glPopMatrix();
    
    // Front left wheel
    glPushMatrix();
    glColor3f(0.1f, 0.1f, 0.1f);  // Black wheels
    glTranslatef(-0.15f, -0.05f, 0.15f);
    glutSolidTorus(0.05f, 0.08f, 10, 10);
    glPopMatrix();
    
    // Front right wheel
    glPushMatrix();
    glColor3f(0.1f, 0.1f, 0.1f);  // Black wheels
    glTranslatef(-0.15f, -0.05f, -0.15f);
    glutSolidTorus(0.05f, 0.08f, 10, 10);
    glPopMatrix();
    
    // Back left wheel
    glPushMatrix();
    glColor3f(0.1f, 0.1f, 0.1f);  // Black wheels
    glTranslatef(0.15f, -0.05f, 0.15f);
    glutSolidTorus(0.05f, 0.08f, 10, 10);
    glPopMatrix();
    
    // Back right wheel
    glPushMatrix();
    glColor3f(0.1f, 0.1f, 0.1f);  // Black wheels
    glTranslatef(0.15f, -0.05f, -0.15f);
    glutSolidTorus(0.05f, 0.08f, 10, 10);
    glPopMatrix();
    
    // Windshield (front window)
    glPushMatrix();
    glColor4f(0.3f, 0.5f, 0.8f, 0.5f);  // Light blue
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glTranslatef(0.0f, 0.2f, 0.11f);
    glScalef(0.25f, 0.08f, 0.01f);
    glutSolidCube(1.0f);
    glDisable(GL_BLEND);
    glPopMatrix();
    
    glPopMatrix();
}

void draw_robots(const Node *robot_starts, int count, const Config *cfg) {
    if (!robot_starts || !cfg) return;
    
    float spacing = 1.0f;
    float scale = 0.5f;
    
    for (int i = 0; i < count; i++) {
        float px = robot_starts[i].x * spacing - (cfg->grid_x * spacing) / 2.0f;
        float py = robot_starts[i].y * spacing - (cfg->grid_y * spacing) / 2.0f;
        float pz = robot_starts[i].z * spacing;
        
        // Draw robot as car with different color
        float r, g, b;
        robot_color_for_index(i, count, &r, &g, &b);
        draw_car(px, py, pz, scale, r, g, b);
    }
}

void draw_path(const Path *path, const Config *cfg, float r, float g, float b) {
    if (!path || !path->valid || !cfg) return;
    
    float spacing = 1.0f;
    
    float cr = r > 1.0f ? 1.0f : r;
    float cg = g > 1.0f ? 1.0f : g;
    float cb = b > 1.0f ? 1.0f : b;
    
    glColor3f(cr, cg, cb);
    glLineWidth(6.0f);  
    glBegin(GL_LINE_STRIP);
    
    for (int i = 0; i < path->length; i++) {
        float px = path->steps[i].x * spacing - (cfg->grid_x * spacing) / 2.0f;
        float py = path->steps[i].y * spacing - (cfg->grid_y * spacing) / 2.0f;
        float pz = path->steps[i].z * spacing + 0.1f;  
        glVertex3f(px, py, pz);
    }
    
    glEnd();
    
    glPointSize(10.0f);
    glColor3f(cr, cg, cb);
    glBegin(GL_POINTS);
    for (int i = 0; i < path->length; i++) {
        float px = path->steps[i].x * spacing - (cfg->grid_x * spacing) / 2.0f;
        float py = path->steps[i].y * spacing - (cfg->grid_y * spacing) / 2.0f;
        float pz = path->steps[i].z * spacing + 0.1f;
        glVertex3f(px, py, pz);
    }
    glEnd();
}

void display(void) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    
    // Set up camera
    glTranslatef(0.0f, 0.0f, -camera_distance);
    glRotatef(camera_angle_x, 1.0f, 0.0f, 0.0f);
    glRotatef(camera_angle_y, 0.0f, 1.0f, 0.0f);
    
    // Enable lighting for better clarity
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
    
    // Draw grid with better visibility
    glPushMatrix();
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    draw_grid(&vis_config);
    glDisable(GL_BLEND);
    glPopMatrix();
    
    // Draw survivors 
    if (vis_survivors) {
        glPushMatrix();
        glDisable(GL_LIGHTING);  
        draw_survivors(vis_survivors, vis_survivor_count, &vis_config);
        glEnable(GL_LIGHTING);
        glPopMatrix();
    }
    
    // Draw robots 
    if (vis_robot_starts) {
        glPushMatrix();
        glDisable(GL_LIGHTING);  
        draw_robots(vis_robot_starts, vis_robot_count, &vis_config);
        glEnable(GL_LIGHTING);
        glPopMatrix();
    }
    
    if (vis_paths) {
        glPushMatrix();
        glDisable(GL_LIGHTING);  
        
        for (int r = 0; r < vis_robot_count; r++) {
            if (vis_paths[r].valid) {
                float r_color, g_color, b_color;
                robot_color_for_index(r, vis_robot_count, &r_color, &g_color, &b_color);
                
                draw_path(&vis_paths[r], &vis_config, r_color, g_color, b_color);
            }
        }
        glEnable(GL_LIGHTING);
        glPopMatrix();
    }
    
    glutSwapBuffers();
}

void reshape(int width, int height) {
    if (height == 0) height = 1;
    
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0f, (float)width / (float)height, 0.1f, 100.0f);
    glMatrixMode(GL_MODELVIEW);
}

void keyboard(unsigned char key, int x, int y) {
    (void)x; (void)y;  
    switch (key) {
        case 'q':
        case 'Q':
        case 27: 
            extern void shutdown_process_pool(void);
            shutdown_process_pool();
            exit(0);
            break;
        case 'r':
        case 'R':
            camera_angle_x = 45.0f;
            camera_angle_y = 45.0f;
            camera_distance = 20.0f;
            break;
    }
    glutPostRedisplay();
}

void mouse(int button, int state, int x, int y) {
    if (state == GLUT_DOWN) {
        mouse_button = button;
        mouse_x = x;
        mouse_y = y;
    }
}

void motion(int x, int y) {
    if (mouse_button == GLUT_LEFT_BUTTON) {
        camera_angle_y += (x - mouse_x) * 0.5f;
        camera_angle_x += (y - mouse_y) * 0.5f;
        if (camera_angle_x > 90.0f) camera_angle_x = 90.0f;
        if (camera_angle_x < -90.0f) camera_angle_x = -90.0f;
    } else if (mouse_button == GLUT_RIGHT_BUTTON) {
        camera_distance += (y - mouse_y) * 0.1f;
        if (camera_distance < 5.0f) camera_distance = 5.0f;
        if (camera_distance > 50.0f) camera_distance = 50.0f;
    }
    mouse_x = x;
    mouse_y = y;
    glutPostRedisplay();
}

void init_visualization(const Config *cfg, const Path *paths, int robot_count,
                       const Survivor *survivors, int survivor_count,
                       const Node *robot_starts) {
    // Copy config
    vis_config = *cfg;
    
    // Allocate and copy paths
    if (vis_paths) free(vis_paths);
    vis_paths = malloc(robot_count * sizeof(Path));
    if (vis_paths && paths) {
        for (int i = 0; i < robot_count; i++) {
            vis_paths[i] = paths[i];
        }
    }
    
    // Allocate and copy survivors
    if (vis_survivors) free(vis_survivors);
    vis_survivors = malloc(survivor_count * sizeof(Survivor));
    if (vis_survivors && survivors) {
        for (int i = 0; i < survivor_count; i++) {
            vis_survivors[i] = survivors[i];
        }
    }
    
    // Allocate and copy robot starts
    if (vis_robot_starts) free(vis_robot_starts);
    vis_robot_starts = malloc(robot_count * sizeof(Node));
    if (vis_robot_starts && robot_starts) {
        for (int i = 0; i < robot_count; i++) {
            vis_robot_starts[i] = robot_starts[i];
        }
    }
    
    vis_robot_count = robot_count;
    vis_survivor_count = survivor_count;
    
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.02f, 0.02f, 0.05f, 1.0f);  
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    
    GLfloat light_pos[] = {15.0f, 15.0f, 15.0f, 1.0f};
    GLfloat light_ambient[] = {0.4f, 0.4f, 0.4f, 1.0f};  
    GLfloat light_diffuse[] = {1.0f, 1.0f, 1.0f, 1.0f};  
    GLfloat light_specular[] = {1.0f, 1.0f, 1.0f, 1.0f};
    
    glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
    
    glShadeModel(GL_SMOOTH);
}

int start_visualization(int argc, char **argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow("3D Grid - Robot Paths Visualization");
    
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    
    printf("\n=== OpenGL Visualization Controls ===\n");
    printf("Mouse Left Button: Rotate camera\n");
    printf("Mouse Right Button: Zoom in/out\n");
    printf("R: Reset camera\n");
    printf("Q or ESC: Quit\n\n");
    
    glutMainLoop();
    return 0;
}
