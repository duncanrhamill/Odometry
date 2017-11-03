import json
import numpy as np

theta = 0
dist = 0
speed = 0.5 # mm/ms
omega = np.pi/2/1000 # rad/ms

with open('path.json', 'r') as f:
    wps = json.loads(f.read())

Waypoints = []

for i, wp in enumerate(wps):
    x = wp[0]
    y = wp[1]
    if (i > 0):
        dx = wp[0] - wps[i - 1][0]
        dy = wp[1] - wps[i - 1][1]
    else:
        dx = 0
        dy = 0
    if (wp[0] == 0):
        t = np.pi/2
    else:
        t = np.tan(wp[1]/wp[0])
    d = (dx**2 + dy**2)**0.5
    Waypoints.append({
        'x': x,
        'y': y,
        't': t,
        'd': d
    })

for i, wp in enumerate(Waypoints):
    nwp = Waypoints[i+1]
    print('At {}, driving to {}'.format(i, i+1))
    while np.abs(theta - wp['t']) > 0.6:
        print("t: {}/{}".format(theta, wp['t']))
        if (theta < wp['t']):
            theta += omega
        else:
            theta -= omega
    while np.abs(dist - wp['d']) > 0.6:
        print("d: {}/{}".format(dist, wp['d']))
        if (dist < wp['d']):
            dist += speed
        else:
            dist -= speed
            