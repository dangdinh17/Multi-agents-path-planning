class AStar():
    def __init__(self, env):
        self.agent_dict = env.agent_dict
        self.admissible_heuristic = env.admissible_heuristic
        self.is_at_goal = env.is_at_goal
        self.get_neighbors = env.get_neighbors

    def construct_path(self, best_way, current):
        total_path = [current]
        while current in best_way.keys():
            current = best_way[current]
            total_path.append(current)
        return total_path[::-1]

    def search(self, agent_name):
        initial_state = self.agent_dict[agent_name]["start"]
        step_cost = 1
        
        arrived_node = set()
        neighbor_nodes = {initial_state}

        best_way = {}

        g_score = {} 
        g_score[initial_state] = 0

        f_score = {} 

        f_score[initial_state] = self.admissible_heuristic(initial_state, agent_name)

        while neighbor_nodes:
            
            temp_dict = {}
            for neighbor_node in neighbor_nodes:
                if neighbor_node not in f_score:
                    f_score[neighbor_node] = float("inf")
                temp_dict[neighbor_node] = f_score[neighbor_node]
            current = min(temp_dict, key=temp_dict.get)

            if self.is_at_goal(current, agent_name):
                return self.construct_path(best_way, current)

            neighbor_nodes -= {current}
            arrived_node |= {current}

            neighbor_list = self.get_neighbors(current)

            for neighbor in neighbor_list:
                if neighbor in arrived_node:
                    continue
                
                tentative_g_score = g_score[current] + step_cost

                if neighbor not in neighbor_nodes:
                    neighbor_nodes |= {neighbor}
                elif tentative_g_score >= g_score[neighbor]:
                    continue

                best_way[neighbor] = current

                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + self.admissible_heuristic(neighbor, agent_name)
        return False

