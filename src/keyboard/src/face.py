import pygame

class Eye:
    def __init__(self, screen, x, y, radius, color):
        self.screen = screen
        self.x = x
        self.y = y
        self.radius = radius
        self.color = color

    def render(self):
        pygame.draw.circle(self.screen, self.color, (self.x, self.y), self.radius)

class Face:
    def __init__(self, screen, color, size):
        self.screen = screen
        self.color = color
        self.size = size
        screen_width, screen_height = self.screen.get_size()
        self.eye_offsets = (screen_width/8, screen_height/8)
        self.left_eye = Eye(self.screen, screen_width/2 - self.eye_offsets[0], screen_height/2 - self.eye_offsets[1], size, color)
        self.right_eye = Eye(self.screen, screen_width/2 + self.eye_offsets[0], screen_height/2 - self.eye_offsets[1], size, color)

    def render(self):
        self.left_eye.render()
        self.right_eye.render()