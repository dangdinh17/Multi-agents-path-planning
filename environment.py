from math import fabs
from itertools import combinations

from a_star import AStar
from env_condition import Conflict, Constraints, EdgeConstraint, VertexConstraint, Location, State

class Environment(object):
    def __init__(self, dimension, agents, obstacles):
        self.dimension = dimension  #kích thước bản đồ
        self.obstacles = obstacles  #tọa độ các vật cản

        self.agents = agents        #số lượng robot
        self.agent_dict = {}

        self.make_agent_dict()

        self.constraints = Constraints()
        self.constraint_dict = {}

        self.a_star = AStar(self)

    #kiểm tra trạng thái của robot tại thời điểm t trong quỹ đạo xác định được
    def get_state(self, agent_name, path, t):
        if t < len(path[agent_name]):
            return path[agent_name][t]
        else:
            return path[agent_name][-1]

    #kiểm tra xem 1 trạng thái có thể di chuyển không
    def state_valid(self, state):
        return state.location.x >= 0 and state.location.x < self.dimension[0] \
            and state.location.y >= 0 and state.location.y < self.dimension[1] \
            and VertexConstraint(state.time, state.location) not in self.constraints.vertex_constraints \
            and (state.location.x, state.location.y) not in self.obstacles

    #kiểm tra độ hợp lệ của chuyển tiếp trạng thái
    def transition_valid(self, state_1, state_2):
        return EdgeConstraint(state_1.time, state_1.location, state_2.location) not in self.constraints.edge_constraints

    #tính toán hàm chi phí hợp lệ
    def admissible_heuristic(self, state, agent_name):
        goal = self.agent_dict[agent_name]["goal"]
        return fabs(state.location.x - goal.location.x) + fabs(state.location.y - goal.location.y)

    #kiểm tra vị trí đích
    def is_at_goal(self, state, agent_name):
        goal_state = self.agent_dict[agent_name]["goal"]
        return state.is_equal_except_time(goal_state)

    #tạo một từ điển lưu thông tin trạng thái của robot
    def make_agent_dict(self):
        for agent in self.agents:
            start_state = State(0, Location(agent['start'][0], agent['start'][1]))
            goal_state = State(0, Location(agent['goal'][0], agent['goal'][1]))

            self.agent_dict.update({agent['name']:{'start':start_state, 'goal':goal_state}})
            
    #xác định các trạng thái có thể di chuyển
    def get_neighbors(self, state):
        neighbors = []

        #trạng thái dừng
        n = State(state.time + 1, state.location)
        if self.state_valid(n):
            neighbors.append(n)
        
        #hàng xóm bên trên
        n = State(state.time + 1, Location(state.location.x, state.location.y+1))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        
        #hàng xóm dưới
        n = State(state.time + 1, Location(state.location.x, state.location.y-1))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        
        #hàng xóm trái
        n = State(state.time + 1, Location(state.location.x-1, state.location.y))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        
        #hàng xóm phải
        n = State(state.time + 1, Location(state.location.x+1, state.location.y))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
            
        return neighbors

    #kiểm tra xung đột
    def get_first_conflict(self, path):
        max_t = max([len(plan) for plan in path.values()])
        result = Conflict()
        for t in range(max_t):
            for agent_1, agent_2 in combinations(path.keys(), 2):
                state_1 = self.get_state(agent_1, path, t)
                state_2 = self.get_state(agent_2, path, t)
                if state_1.is_equal_except_time(state_2):
                    result.time = t
                    result.type = Conflict.VERTEX
                    result.location_1 = state_1.location
                    result.agent_1 = agent_1
                    result.agent_2 = agent_2
                    return result

            for agent_1, agent_2 in combinations(path.keys(), 2):
                state_1a = self.get_state(agent_1, path, t)
                state_1b = self.get_state(agent_1, path, t+1)

                state_2a = self.get_state(agent_2, path, t)
                state_2b = self.get_state(agent_2, path, t+1)

                if state_1a.is_equal_except_time(state_2b) and state_1b.is_equal_except_time(state_2a):
                    result.time = t
                    result.type = Conflict.EDGE
                    result.agent_1 = agent_1
                    result.agent_2 = agent_2
                    result.location_1 = state_1a.location
                    result.location_2 = state_1b.location
                    return result
        return False

    #tạo ra các ràng buộc từ xung đột tìm được
    def create_constraints_from_conflict(self, conflict):
        constraint_dict = {}
        if conflict.type == Conflict.VERTEX:
            v_constraint = VertexConstraint(conflict.time, conflict.location_1)
            constraint = Constraints()
            constraint.vertex_constraints |= {v_constraint}
            constraint_dict[conflict.agent_1] = constraint
            constraint_dict[conflict.agent_2] = constraint

        elif conflict.type == Conflict.EDGE:
            constraint1 = Constraints()
            constraint2 = Constraints()

            e_constraint1 = EdgeConstraint(conflict.time, conflict.location_1, conflict.location_2)
            e_constraint2 = EdgeConstraint(conflict.time, conflict.location_2, conflict.location_1)

            constraint1.edge_constraints |= {e_constraint1}
            constraint2.edge_constraints |= {e_constraint2}

            constraint_dict[conflict.agent_1] = constraint1
            constraint_dict[conflict.agent_2] = constraint2

        return constraint_dict

    #tìm kiểm đường đi bằng A*
    def compute_path(self):
        path = {}
        for agent in self.agent_dict.keys():
            if agent not in self.constraint_dict:
                self.constraint_dict[agent] = Constraints()
            self.constraints = self.constraint_dict[agent]
            
            local_path = self.a_star.search(agent)
            if not local_path:
                return False
            path.update({agent:local_path})
        return path

    #tính toán chi phí của đường đi
    def compute_path_cost(self, path):
        return sum([len(path) for path in path.values()])

#tạo node từ hàng đợi ưu tiên cho thuật toán A*
class PriorityNode(object):
    def __init__(self):
        self.path = {}
        self.constraint_dict = {}
        self.cost = 0

    #so sánh 2 node ưu tiên về quỹ đạo và chi phí
    def __eq__(self, other):
        if not isinstance(other, type(self)): return NotImplemented
        return self.path == other.path and self.cost == other.cost

    #tạo bảng băm cho chi phí
    def __hash__(self):
        return hash((self.cost))

    #sắp sếp theo chi phí
    def __lt__(self, other):
        return self.cost < other.cost
