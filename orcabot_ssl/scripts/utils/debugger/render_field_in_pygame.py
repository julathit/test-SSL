import pygame
import random

def render(data):
    # Pygame setup
    pygame.init()
    window_width, window_height = 900, 600  # Pygame window size
    screen = pygame.display.set_mode((window_width, window_height))
    pygame.display.set_caption("Rendering Boxes")

    # Field size coordinates (from the prompt)
    field_x_min, field_x_max = -4500, 4500
    field_y_min, field_y_max = -3000, 3000

    # Your field data
    field_data = data
    # Normalize function to convert field coordinates to screen coordinates
    def normalize_coordinates(x, y, field_x_min, field_x_max, field_y_min, field_y_max, window_width, window_height):
        # Normalize x
        screen_x = (x - field_x_min) / (field_x_max - field_x_min) * window_width
        # Normalize y
        screen_y = (-y - field_y_min) / (field_y_max - field_y_min) * window_height
        return screen_x, screen_y

    # Function to generate random colors
    def random_color():
        return (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))

    # Function to draw the boxes and labels
    def draw_boxes_and_labels():
        screen.fill((255, 255, 255))  # Fill background with white

        font = pygame.font.SysFont(None, 24)  # Font for labels

        # Iterate over the field_data dictionary and draw boxes
        for label, coords in field_data.items():
            # Get screen coordinates for the min/max corners
            x_min, y_max = normalize_coordinates(coords['x_min'], coords['y_min'], field_x_min, field_x_max, field_y_min, field_y_max, window_width, window_height)
            x_max, y_min = normalize_coordinates(coords['x_max'], coords['y_max'], field_x_min, field_x_max, field_y_min, field_y_max, window_width, window_height)

            # Generate a random color for the box
            box_color = random_color()

            # Draw the box (as a rectangle)
            pygame.draw.rect(screen, box_color, pygame.Rect(x_min, y_min, x_max - x_min, y_max - y_min), 2)  # Draw the rectangle

            # Render the label
            label_text = font.render(label, True, (0, 0, 0))  # Black text
            screen.blit(label_text, (x_min + 15, y_min + 15))  # Place the label under the box

        pygame.display.update()

    # Main loop

    draw_boxes_and_labels()


    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False


    pygame.quit()