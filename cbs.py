import yaml
from copy import deepcopy

from environment import Constraints, Environment, PriorityNode

class CBS(object):
    def __init__(self, environment):
        self.env = environment
        self.neighbor_nodes = set()
        self.arrived_set = set()
    def search(self):
        start = PriorityNode()
        start.constraint_dict = {}
        for agent in self.env.agent_dict.keys():
            start.constraint_dict[agent] = Constraints()
        start.path = self.env.compute_path()
        if not start.path:
            return {}
        start.cost = self.env.compute_path_cost(start.path)

        self.neighbor_nodes |= {start}

        while self.neighbor_nodes:
            P = min(self.neighbor_nodes)
            self.neighbor_nodes -= {P}
            self.arrived_set |= {P}

            self.env.constraint_dict = P.constraint_dict
            conflict_dict = self.env.get_first_conflict(P.path)
            if not conflict_dict:
                print("path found")
                return self.generate_plan(P.path)

            constraint_dict = self.env.create_constraints_from_conflict(conflict_dict)

            for agent in constraint_dict.keys():
                new_node = deepcopy(P)
                new_node.constraint_dict[agent].add_constraint(constraint_dict[agent])

                self.env.constraint_dict = new_node.constraint_dict
                new_node.path = self.env.compute_path()
                if not new_node.path:
                    continue
                new_node.cost = self.env.compute_path_cost(new_node.path)

                if new_node not in self.arrived_set:
                    self.neighbor_nodes |= {new_node}

        return {}

    def generate_plan(self, path):
        plan = {}
        for agent, path in path.items():
            path_dict_list = [{'t':state.time, 'x':state.location.x, 'y':state.location.y} for state in path]
            plan[agent] = path_dict_list
        return plan

def main():
    param = 'input.yaml'
    output_file = 'output.yaml'
    # Read from input file
    with open(param, 'r') as param_file:
        try:
            param = yaml.load(param_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    dimension = param["map"]["dimensions"]
    obstacles = param["map"]["obstacles"]
    agents = param['agents']

    env = Environment(dimension, agents, obstacles)

    # Searching
    cbs = CBS(env)
    path = cbs.search()
    if not path:
        print(" path not found!" )
        return

    # Write to output file
    output = dict()
    output["path planning"] = path
    output["cost"] = env.compute_path_cost(path)
    with open(output_file, 'w') as output_yaml:
        yaml.safe_dump(output, output_yaml)

if __name__ == "__main__":
    main()
