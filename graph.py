import matplotlib.pyplot as plt
import numpy as np

episodes = []
steps = []

with open('output copy.txt', 'r') as f:
    for line in f:
        if 'Episode' in line:
            episode = line.strip()
            episodes.append(int(episode.split(':')[0].split()[-1]))
            steps.append(int(episode.split(':')[-1].split()[0]))

p = np.polyfit(episodes, steps, 4)
f = np.poly1d(p)
t = np.linspace(0, 400, 100)

plt.plot(episodes, steps, 'o', color = 'orange', label='# Steps')
plt.plot(t, f(t), '-', color = 'green', linewidth=3, label='Mean Steps w.r.t # Episodes')
plt.xlabel('Episode')
plt.ylabel('Number of steps')
plt.title('Training Performance')
plt.legend()
plt.show()
print(np.mean(steps))
print(np.std(steps))
print(np.max(steps))
print(np.min(steps))