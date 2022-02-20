import numpy as np

D = 1
U = 2
R = 4
L = 8
DX = {D:1,R:0}
DY = {D:0,R:1}
OP = {D:U, R:L}

def kruskal(n,m):
    edges = np.array([(i,j,D) for i in range(n-1) for j in range(m)] + \
            [(i,j,R) for i in range(n) for j in range(m-1)])
    sets = {(i,j):{(i,j)} for i in range(n) for j in range(m)}
    edges = np.random.permutation(edges)
    grid = np.array([[0 for j in range(m)] for i in range(n)])
    while edges.size > 0:
        x, y, dir, edges  = *edges[-1], edges[:-1]
        nx, ny = x+DX[dir], y+DY[dir]
        # print(sets)
        # print(x,y,dir)
        # print(nx,ny)
        if sets[(x,y)]!=sets[(nx,ny)]:
            n_set = sets[(x,y)] | sets[(nx,ny)]
            for t in n_set:
                sets[t] = n_set
            print(x)
            grid[x][y] |= dir
            grid[nx][ny] |= OP[dir]
    return grid