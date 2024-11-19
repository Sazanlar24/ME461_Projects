import pygame
from grid import Grid

def SearchSnakeP1(nWidth = 10, nHeight = 10, nObjects = 20):

    if nWidth > 0 and nHeight > 0:
        myGrid = Grid(nWidth, nHeight, nObjects)
    else:
        print("oops...")

    clock = pygame.time.Clock()
    running = True

    while running:
        myGrid.screen.fill(myGrid.BLACK)
        myGrid.draw_grid()
        myGrid.draw_objects()

        if myGrid.ready_for_search:
            # SEARCH KODU BAŞLANGIÇ NOKTASI
            pass
        
        state = myGrid.event_handler()    

        if state is False:
            running = False

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":

    SearchSnakeP1()