import random
import matplotlib.pyplot as plt
import heapq

class CityGrid:
    def __init__(self, N, M, obstacle_density=0.3):
        self.N = N
        self.M = M
        self.grid = [[0] * M for _ in range(N)]

        for i in range(N):
            for j in range(M):
                if random.random() < obstacle_density:
                    self.grid[i][j] = 1

        self.towers = []

    def place_tower(self, x, y, R):
        if self.grid[x][y] != 1:
            self.towers.append((x, y, R))

    def place_optimal_towers(self, R):
        for i in range(self.N):
            for j in range(self.M):
                if self.grid[i][j] != 1:
                    is_covered = False
                    for tower in self.towers:
                        x, y, _ = tower
                        if abs(x - i) <= R and abs(y - j) <= R:
                            is_covered = True
                            break
                    if not is_covered:
                        self.towers.append((i, j, R))

    def find_reliable_path(self, start, end):
        graph = {}
        for i in range(len(self.towers)):
            graph[i] = []
            x1, y1, _ = self.towers[i]
            for j in range(len(self.towers)):
                if i != j:
                    x2, y2, _ = self.towers[j]
                    distance = abs(x1 - x2) + abs(y1 - y2)
                    if distance <= self.towers[i][2]:
                        reliability = 1 / (distance + 1)
                        graph[i].append((j, reliability))

        def dijkstra(graph, start, end):
            min_reliabilities = {node: float('inf') for node in graph}
            min_reliabilities[start] = 1
            previous_nodes = {}
            priority_queue = [(1, start)]

            while priority_queue:
                current_reliability, current_node = heapq.heappop(priority_queue)

                if current_reliability < min_reliabilities[current_node]:
                    continue

                for neighbor, edge_reliability in graph[current_node]:
                    reliability = current_reliability * edge_reliability
                    if reliability > min_reliabilities[neighbor]:
                        continue
                    min_reliabilities[neighbor] = reliability
                    previous_nodes[neighbor] = current_node
                    heapq.heappush(priority_queue, (reliability, neighbor))

            path = []
            while end is not None:
                path.insert(0, end)
                end = previous_nodes.get(end)
            return path

        def calculate_reliability(path):
            total_reliability = 1
            for i in range(len(path) - 1):
                node1 = path[i]
                node2 = path[i + 1]
                for neighbor, reliability in graph[node1]:
                    if neighbor == node2:
                        total_reliability *= reliability
                        break
            return total_reliability

        start_node = self.towers.index(start)
        end_node = self.towers.index(end)
        path_indices = dijkstra(graph, start_node, end_node)
        path_towers = [self.towers[i] for i in path_indices]
        path_reliability = calculate_reliability(path_indices)

        return path_towers, path_reliability

    def visualize_grid(self):
        fig, ax = plt.subplots()
        for i in range(self.N):
            for j in range(self.M):
                if self.grid[i][j] == 1:
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, facecolor='green'))
        ax.set_aspect('equal', 'box')
        plt.xlim(0, self.M)
        plt.ylim(0, self.N)

    def visualize_towers(self):
        for tower in self.towers:
            x, y, R = tower
            circle = plt.Circle((y + 0.5, x + 0.5), R, color='orange', alpha=0.3)
            plt.gca().add_patch(circle)

    def visualize_paths(self):
        for i in range(len(self.towers)):
            for j in range(i + 1, len(self.towers)):
                start = (self.towers[i][0] + 0.5, self.towers[i][1] + 0.5)
                end = (self.towers[j][0] + 0.5, self.towers[j][1] + 0.5)
                plt.plot([start[1], end[1]], [start[0], end[0]], 'r-', alpha=0.5)

city = CityGrid(20, 20, obstacle_density=0.3)
city.place_optimal_towers(R=2)
city.visualize_grid()
city.visualize_towers()
path_towers, path_reliability = city.find_reliable_path(city.towers[0], city.towers[1])
city.visualize_paths()
print(f"Most Reliable Path: {path_towers}")
print(f"Path Reliability: {path_reliability:.2f}")
plt.show()