#! /usr/bin/env python3

import matplotlib.pyplot as plt

t = []
v = []

with open("/tmp/velocity.txt") as f:
    for l in f.readlines():
        a = l.split()
        if len(a) < 2:
            continue
        t.append(float(a[0]))
        v.append(float(a[1]))

plt.plot(t, v)

plt.ylabel('velocity')
plt.xlabel('simulation_time')
plt.show()