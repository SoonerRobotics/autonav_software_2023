import matplotlib as mpl
from matplotlib import pyplot as plt

def setup_pyplot():
    plt.ion()
    plt.show()

# Draws pure pursuit information to pyplot
def draw_pp(agent_pos, lookahead_pos, waypoints, xlims=[1,-1], ylims=[-1,1], fig_num=1):
    plt.figure(fig_num)
    plt.clf()
    plt.xlim([agent_pos[1] + xlims[0], agent_pos[1] + xlims[1]])
    plt.ylim([agent_pos[0] + ylims[0], agent_pos[0] + ylims[1]])

    if waypoints:
        for point in waypoints:
            plt.plot(point[1], point[0], '.', markersize=6)

    if agent_pos:
        plt.plot(agent_pos[1], agent_pos[0], 's', markersize=8)

    if lookahead_pos:
        plt.plot(lookahead_pos[1], lookahead_pos[0], 'x', markersize=8)

    plt.draw()
    plt.pause(0.00000000001)