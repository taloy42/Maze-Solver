from mimetypes import init, inited
import pygame
import math
from queue import PriorityQueue
from mazegen import kruskal,U,D,L,R

WIDTH = 800
WIN = pygame.display.set_mode((WIDTH,WIDTH))
pygame.display.set_caption("A* Path Finding Algorithm")

CLOSED = (255, 0, 0)
OPEN = (0, 255, 0)
BLUE = (0, 255, 0)
YELLOW = (255, 255, 0)
BLANK = (255, 255, 255)
BARRIER = (0, 0, 0)
PATH = (255, 255, 0)# (128, 0, 128)
START = (255, 165 ,0)
SEPARATOR = (128, 128, 128)
END = (64, 224, 208)

class Node:
    def __init__(self, row, col, width, total_rows) -> None:
        self.row, self.col = row, col
        self.x = row*width
        self.y = col*width
        self.color = BLANK
        self.neighbors = []
        self.width = width
        self.total_rows = total_rows
        self.pi = None

    def get_pos(self):
        return self.row, self.col
    
    def is_closed(self):
        return self.color == CLOSED
    
    def is_open(self):
        return self.color == OPEN
    
    def is_barrier(self):
        return self.color == BARRIER

    def is_start(self):
        return self.color == START

    def is_end(self):
        return self.color == END
    
    def is_path(self):
        return self.color == PATH
    
    def is_auxiliary(self):
        return self.is_closed() or self.is_open() or self.is_path()
    
    def reset(self):
        self.color = BLANK
    
    def make_closed(self):
        self.color = CLOSED
    
    def make_open(self):
        self.color = OPEN
    
    def make_barrier(self):
        self.color = BARRIER

    def make_start(self):
        self.color = START

    def make_end(self):
        self.color = END
    
    def make_path(self):
        self.color = PATH
    
    def draw(self, win):
        pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))
    
    def update_neighbors(self, grid):
        self.neighbors = []
        for i in [0,1]:
            for j in [-1,1]:
                x = self.row + (1-i)*j
                y = self.col + i*j
                # print(f"cur : ({self.row},{self.col})\tnew : ({x},{y})")
                if (0<=x<self.total_rows 
                        and 
                    0<=y<self.total_rows):
                    cur = grid[x][y]
                    if not cur.is_barrier():
                        self.neighbors.append(cur)
    
    def __lt__(self, other):
        return False


def h(p1, p2):
    x1, y1 = p1
    x2, y2 = p2

    return abs(x1-x2) + abs(y1-y2)

def reconstruct_path(current, draw_grid):
    current.make_end()
    while current.pi is not None:
        current = current.pi
        current.make_path()
        draw_grid()
    current.make_start()


def astar_alg(draw_grid, grid, start, end, state = None):
    if state is None:
        for row in grid:
            for node in row:
                if node.is_auxiliary():
                    node.reset()
        end.make_end()
        count = 0
        open_set = PriorityQueue()
        open_set.put((0, count, start)) #score, count, node
        came_from = {}
        g_score = {node: float("inf") for row in grid for node in row}
        g_score[start] = 0

        f_score = {node: float("inf") for row in grid for node in row}
        f_score[start] = h(start.get_pos(), end.get_pos())

        open_set_hash = {start}
    else:
        grid,start,end,count,open_set,open_set_hash,g_score,f_score = state

    while not open_set.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_p: #pause
                state = (
                    grid,
                    start, end,
                    count,
                    open_set, open_set_hash,
                    g_score, f_score
                )
                return state
        
        current = open_set.get()[2] #current node
        open_set_hash.remove(current)

        if current == end:
            reconstruct_path(end, draw_grid)
            return True
        
        for neighbor in current.neighbors:
            temp_g_score = g_score[current]+1
            if temp_g_score < g_score[neighbor]:
                g_score[neighbor] = temp_g_score
                neighbor.pi = current
                f_score[neighbor] = temp_g_score + h(neighbor.get_pos(), end.get_pos())
                if neighbor not in open_set_hash:
                    count += 1
                    open_set.put((f_score[neighbor], count, neighbor))
                    open_set_hash.add(neighbor)
                    neighbor.make_open()
        draw_grid()

        if current != start:
            current.make_closed()
    return False


def make_grid(rows, width):
    gap = width//rows
    grid = [[Node(i,j,gap,rows) for j in range(rows)] for i in range(rows)]
    return grid

def gen_maze_grid(rows, width):
    gap = width//rows
    grid = [[Node(i,j,gap,rows) for j in range(rows)] for i in range(rows)]
    for i in range(rows):
        for j in range(rows):
            if i%2==1 and j%2 == 1 :
                grid[i][j].make_barrier()
            if i%2==0 and j%2 ==0:
                grid[i][j].reset() 
        
    rand_maze = kruskal(rows//2, rows//2)

    for i in range(rows//2):
        for j in range(rows//2):
            cur = rand_maze[i][j]
            cur_i = 2*i
            cur_j = 2*j
            # if cur&U and cur_i-1>=0:
            #     grid[cur_i-1][cur_j].reset()
            if (not cur&D) and cur_i+1<rows:
                grid[cur_i+1][cur_j].make_barrier()
            # if cur&L and cur_j-1>=0:
            #     grid[cur_i][cur_j-1].reset()
            if (not cur&R) and cur_j+1<rows:
                grid[cur_i][cur_j+1].make_barrier()
    return grid



def draw_gridlines(win, rows, width):
    gap = width // rows
    for i in range(rows):
        line_row_start = (0,i*gap)
        line_row_end = (width,i*gap)
        pygame.draw.line(win,SEPARATOR,line_row_start,line_row_end)
        line_col_start = (i*gap,0)
        line_col_end = (i*gap,width)
        pygame.draw.line(win,SEPARATOR,line_col_start,line_col_end)

def draw_grid(win, grid, rows, width):
    win.fill(BLANK)

    for row in grid:
        for node in row:
            node.draw(win)
    draw_gridlines(win, rows, width)
    pygame.display.update()

def get_clicked_pos(pos, rows, width):
    gap = width // rows
    
    y, x = pos
    
    row = y // gap
    col = x // gap

    return row, col

def main_loop(win, width):
    ROWS = 100
    # grid = make_grid(ROWS, width)
    grid = make_grid(ROWS, width)

    start = None
    end = None

    state = None

    run = True
    started = False

    paused = False
    while run:
        draw_grid(win, grid,ROWS, width)
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_q):
                run = False
                break
            
            if started:
                continue

            if pygame.mouse.get_pressed()[0]: # left click
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, width)
                node = grid[row][col]
                if start is None and not node.is_end():
                    start = node
                    start.make_start()
                elif end is None and not node.is_start():
                    end = node
                    end.make_end()
                elif not (node.is_end() or node.is_start()):
                    node.make_barrier() 
            elif pygame.mouse.get_pressed()[2]: #right click
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, width)
                node = grid[row][col]

                if node.is_start():
                    start = None
                if node.is_end():
                    end = None

                node.reset()
                

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_c:
                    start = None
                    end = None
                    state = None
                    grid = make_grid(ROWS, width)
                if start and end \
                    and event.key == pygame.K_SPACE and not started:
                    for row in grid:
                        for node in row:
                            node.update_neighbors(grid)
                    state = astar_alg(lambda: draw_grid(win, grid, ROWS, width), grid, start, end)
                if event.key == pygame.K_p:
                    if not (state is True or state is False):
                        astar_alg(lambda: draw_grid(win, grid, ROWS, width), grid, start, end, state)
                if event.key == pygame.K_r:
                    grid = gen_maze_grid(ROWS, width)
                    start = grid[0][0]
                    start.make_start()
                    end = grid[ROWS-1][ROWS-2]
                    end.make_end()
                    state = None
    pygame.quit()



main_loop(WIN, WIDTH)