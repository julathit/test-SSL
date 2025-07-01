// astar.c
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <limits.h>

// Define a struct to represent a node in the grid
typedef struct {
    int row;
    int col;
    int cost; // Cost from start to this node
    int priority; // cost + heuristic
    struct Node* prev; // Pointer to the previous node in the path
} Node;

// Function to calculate the heuristic (Manhattan distance)
int heuristic(int row1, int col1, int row2, int col2) {
    return (int)sqrt(pow(row1 - row2, 2) + pow(col1 - col2, 2));
}

// Function to implement A* algorithm
Node* astar(int* grid, int rows, int cols, int start_row, int start_col, int end_row, int end_col, int* path_length, int* visited_count) {
    // Create open set (priority queue) and visited set.  For the open set, we'll use a simple array
    // and just find the minimum cost node in it at each iteration.  A proper heap would be faster for large grids.
    Node* open_set[rows * cols];
    int open_set_size = 0;
    bool visited[rows][cols];
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            visited[i][j] = false;
        }
    }

    // Create start and end nodes
    Node* start_node = (Node*)malloc(sizeof(Node));
    start_node->row = start_row;
    start_node->col = start_col;
    start_node->cost = 0;
    start_node->priority = heuristic(start_row, start_col, end_row, end_col);
    start_node->prev = NULL;

    Node* end_node = (Node*)malloc(sizeof(Node));
    end_node->row = end_row;
    end_node->col = end_col;
    end_node->cost = INT_MAX; // We don't know the cost to reach the end node yet.
    end_node->priority = 0;
    end_node->prev = NULL;


    open_set[open_set_size++] = start_node;

    Node* current = NULL; //stores the current node

    // Main loop
    while (open_set_size > 0) {
        // Find the node with the lowest priority in the open set
        int min_index = 0;
        int min_priority = open_set[0]->priority;
        for (int i = 1; i < open_set_size; i++) {
            if (open_set[i]->priority < min_priority) {
                min_index = i;
                min_priority = open_set[i]->priority;
            }
        }
        current = open_set[min_index]; //get the node with the smallest priority.

        // Remove the node from the open set
        for (int i = min_index; i < open_set_size - 1; i++) {
            open_set[i] = open_set[i + 1];
        }
        open_set_size--;

        visited[current->row][current->col] = true;
        (*visited_count)++; // Increment visited node count

        if (current->row == end_row && current->col == end_col) {
            break; // Found the end node
        }

        // Explore neighbors (up, down, left, right)
        int dr[] = {0, 0, 1, -1};
        int dc[] = {1, -1, 0, 0};
        for (int i = 0; i < 4; i++) {
            int neighbor_row = current->row + dr[i];
            int neighbor_col = current->col + dc[i];

            // Check if the neighbor is within bounds and is not an obstacle
            if (neighbor_row >= 0 && neighbor_row < rows && neighbor_col >= 0 && neighbor_col < cols && grid[neighbor_row * cols + neighbor_col] == 0 && !visited[neighbor_row][neighbor_col]) {
                Node* neighbor = (Node*)malloc(sizeof(Node));
                neighbor->row = neighbor_row;
                neighbor->col = neighbor_col;
                int new_cost = current->cost + 1;  // Cost to move to neighbor is 1
                neighbor->cost = new_cost;
                neighbor->priority = new_cost + heuristic(neighbor_row, neighbor_col, end_row, end_col);
                neighbor->prev = current;

                bool found_in_open_set = false;
                for(int j = 0; j < open_set_size; j++){
                    if(open_set[j]->row == neighbor_row && open_set[j]->col == neighbor_col){
                        found_in_open_set = true;
                        if(new_cost < open_set[j]->cost){
                            open_set[j]->cost = new_cost;
                            open_set[j]->priority = new_cost + heuristic(neighbor_row, neighbor_col, end_row, end_col);
                            open_set[j]->prev = current;
                        }
                        break;
                    }
                }

                if (!found_in_open_set) {
                    open_set[open_set_size++] = neighbor;
                }
            }
        }
    }

    // If the end node was not found, return NULL
    if (current->row != end_row || current->col != end_col) {
        return NULL;
    }

    // Reconstruct the path from the end node to the start node
    Node* path = NULL;
    Node* current_node = current;
    int length = 0;
    while (current_node != NULL) {
        length++;
        Node* temp = (Node*)malloc(sizeof(Node));
        temp->row = current_node->row;
        temp->col = current_node->col;
        temp->prev = path;  // Build the linked list in reverse
        path = temp;
        current_node = current_node->prev;
    }
    *path_length = length;
    return path; //return the path
}


// Function to free the memory allocated for the path
void free_path(Node* path) {
    Node* current = path;
    while (current != NULL) {
        Node* next = current->prev;
        free(current);
        current = next;
    }
}
