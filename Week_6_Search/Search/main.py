import pygame
from grid import Grid

def Snake_Search(nWidth = 15, nHeight = 9, nObjects = 25):

    if nWidth > 0 and nHeight > 0:
        myGrid = Grid(nWidth, nHeight, nObjects)
    else:
        print("oops...")

    clock = pygame.time.Clock()
    running = True

    while running:
        myGrid.draw_grid()
        myGrid.draw_objects()

        if myGrid.ready_for_search:
            myGrid.start_search()
            myGrid.ready_for_search = False  # Prevent re-triggering the search on subsequent frames
            pass
        
        state = myGrid.event_handler()    

        if state is False:
            running = False

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":

    Snake_Search()