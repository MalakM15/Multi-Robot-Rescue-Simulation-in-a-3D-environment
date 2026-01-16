#ifndef VISUALIZE_H
#define VISUALIZE_H

/**
 * @file visualize.h
 * @brief OpenGL 3D Visualization for Grid, Robots, Survivors, and Paths
 * 
 * This module provides 3D visualization using OpenGL/GLUT to display:
 * - 3D Grid: White boxes (free cells) and Gray boxes (obstacles)
 * - Robots: Blue boxes representing robot starting positions
 * - Survivors: Red human figures representing people to be rescued
 * - Paths: Colored lines showing robot paths to survivors
 * 
 * Visualization Legend:
 * - White boxes: Free/empty grid cells (traversable)
 * - Gray boxes: Obstacles/debris (blocked cells)
 * - Blue boxes: Robots
 * - Red human figures: Survivors (people to rescue)
 * - Colored lines: Robot paths (different color per robot)
 * 
 * Controls:
 * - Left Mouse Button: Rotate camera
 * - Right Mouse Button: Zoom in/out
 * - R: Reset camera to default position
 * - Q or ESC: Quit visualization
 */

#include "config.h"
#include "astar.h"

/**
 * @brief Initialize visualization data
 * 
 * Copies the grid configuration, paths, survivors, and robot positions
 * into internal storage for rendering.
 * 
 * @param cfg Grid configuration
 * @param paths Array of robot paths (one per robot)
 * @param robot_count Number of robots
 * @param survivors Array of survivor positions
 * @param survivor_count Number of survivors
 * @param robot_starts Array of robot starting positions
 */
void init_visualization(const Config *cfg, const Path *paths, int robot_count,
                       const Survivor *survivors, int survivor_count,
                       const Node *robot_starts);

/**
 * @brief Start OpenGL visualization window
 * 
 * Creates and displays the OpenGL window with interactive 3D visualization.
 * This function blocks until the window is closed.
 * 
 * @param argc Command line argument count (for GLUT)
 * @param argv Command line arguments (for GLUT)
 * @return 0 on successful completion
 */
int start_visualization(int argc, char **argv);

#endif

