#!/usr/bin/env python
import rospy
import pygame
import tkinter as tk
import os


from face import Face

UBUNTU_PURPLE = (48, 10, 36)
WHITE = (255, 255, 255)

# Run until the user asks to quit
def main():
    pygame.init()

    # screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
    # screen = pygame.display.set_mode([500, 500])

    # display_info = pygame.display.Info()
    # screen = pygame.display.set_mode((display_info.current_w, display_info.current_h))
    # screen_width, screen_height = screen.get_size()

    root = tk.Tk()
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    window_width = screen_width * 1.0
    window_height = screen_height * 0.95

    screen_x = 0
    screen_y = 0
    os.environ["SDL_VIDEO_WINDOW_POS"] = f"{screen_x},{screen_y}"

    screen = pygame.display.set_mode((window_width, window_height))

    rospy.init_node("gui", anonymous=True)
    rospy.loginfo("ROS GUI Python Node")
    rate = rospy.Rate(10)

    face = Face(screen, WHITE, 75)
    
    while not rospy.is_shutdown():
        pygame.time.delay(100)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                break

        # Fill the background with white
        screen.fill(UBUNTU_PURPLE)
        
        face.render()

        # Flip the display
        pygame.display.flip()
        pygame.display.update()
        
        rate.sleep()

    pygame.quit()


if __name__=="__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass