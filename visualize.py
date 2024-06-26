import yaml
from matplotlib.patches import Circle, Rectangle
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation

Colors = ['orange', 'yellow', 'green']


class Animation:
  def __init__(self, map, schedule):
    self.map = map
    self.schedule = schedule
    self.combined_schedule = {}
    self.combined_schedule.update(self.schedule["path planning"])

    aspect = map["map"]["dimensions"][0] / map["map"]["dimensions"][1]

    self.fig = plt.figure(frameon=False, figsize=(6 * aspect, 6))
    self.ax = self.fig.add_subplot(111, aspect='equal')
    self.fig.subplots_adjust(left=0,right=1,bottom=0,top=1, wspace=None, hspace=None)
    # self.ax.set_frame_on(False)

    self.patches = []
    self.artists = []
    self.agents = dict()
    self.agent_names = dict()
    # create boundary patch
    xmin = -0.5
    ymin = -0.5
    xmax = map["map"]["dimensions"][0] - 0.5
    ymax = map["map"]["dimensions"][1] - 0.5

    plt.xlim(xmin, xmax)
    plt.ylim(ymin, ymax)

    self.patches.append(Rectangle((xmin, ymin), xmax - xmin, ymax - ymin, facecolor='none', edgecolor='red'))
    #thêm vật cản
    for o in map["map"]["obstacles"]:
      x, y = o[0], o[1]
      self.patches.append(Rectangle((x - 0.5, y - 0.5), 1, 1, facecolor='red', edgecolor='red'))

    # create agents:
    self.T = 0
    #thêm điểm đích
    for d, i in zip(map["agents"], range(0, len(map["agents"]))):
      goal_x = d["goal"][0] - 0.25
      goal_y = d["goal"][1] - 0.25
      self.patches.append(Rectangle((goal_x, goal_y), 0.5, 0.5, facecolor=Colors[1], edgecolor='black', alpha=0.5))
      
      name = d['name']
      goal_name = self.ax.text(goal_x + 0.25, goal_y + 0.25, name.replace('agent', ''), color='black', ha='center', va='center')
      self.artists.append(goal_name)
    
    #thêm robot
    for d, i in zip(map["agents"], range(0, len(map["agents"]))):
      name = d["name"]
      self.agents[name] = Circle((d["start"][0], d["start"][1]), 0.3, facecolor=Colors[2], edgecolor='black')
      self.agents[name].original_face_color = Colors[2]
      self.patches.append(self.agents[name])
      
      self.T = max(self.T, schedule["path planning"][name][-1]["t"])
      self.agent_names[name] = self.ax.text(d["start"][0], d["start"][1], name.replace('agent', ''))
      self.agent_names[name].set_horizontalalignment('center')
      self.agent_names[name].set_verticalalignment('center')
      self.artists.append(self.agent_names[name])

    self.anim = animation.FuncAnimation(self.fig, self.animate_func,
                               init_func=self.init_func,
                               frames=int(self.T+1) * 10,
                               interval=100,
                               blit=True)

  def save(self, file_name, speed):
    self.anim.save(
      file_name,
      "ffmpeg",
      fps=10 * speed,
      dpi=200)

  def show(self):
    plt.show()

  def init_func(self):
    for p in self.patches:
      self.ax.add_patch(p)
    for a in self.artists:
      self.ax.add_artist(a)
    return self.patches + self.artists

  def animate_func(self, i):
    for agent_name, agent in self.combined_schedule.items():
      pos = self.getState(i / 10, agent)
      p = (pos[0], pos[1])
      self.agents[agent_name].center = p
      self.agent_names[agent_name].set_position(p)

    # reset all colors
    for _,agent in self.agents.items():
      agent.set_facecolor(agent.original_face_color)

    # check drive-drive collisions
    agents_array = [agent for _,agent in self.agents.items()]
    for i in range(0, len(agents_array)):
      for j in range(i+1, len(agents_array)):
        d1 = agents_array[i]
        d2 = agents_array[j]
        pos1 = np.array(d1.center)
        pos2 = np.array(d2.center)
        if np.linalg.norm(pos1 - pos2) < 0.7:
          d1.set_facecolor('red')
          d2.set_facecolor('red')
          print("COLLISION! (agent-agent) ({}, {})".format(i, j))

    return self.patches + self.artists


  def getState(self, t, d):
    idx = 0
    while idx < len(d) and d[idx]["t"] < t:
      idx += 1
    if idx == 0:
      return np.array([float(d[0]["x"]), float(d[0]["y"])])
    elif idx < len(d):
      posLast = np.array([float(d[idx-1]["x"]), float(d[idx-1]["y"])])
      posNext = np.array([float(d[idx]["x"]), float(d[idx]["y"])])
    else:
      return np.array([float(d[-1]["x"]), float(d[-1]["y"])])
    dt = d[idx]["t"] - d[idx-1]["t"]
    t = (t - d[idx-1]["t"]) / dt
    pos = (posNext - posLast) * t + posLast
    return pos



if __name__ == "__main__":
  map = 'map8x8.yaml'
  schedule = 'output.yaml'
  video = 'simulation.gif'
  speed = 3
  with open(map) as map_file:
    map = yaml.load(map_file, Loader=yaml.FullLoader)

  with open(schedule) as states_file:
    schedule = yaml.load(states_file, Loader=yaml.FullLoader)

  animation = Animation(map, schedule)

  animation.save(video, speed)
  animation.show()
