from simple_launch import SimpleLauncher


def generate_launch_description():

    sl = SimpleLauncher(use_sim_time=False)

    sl.node("turtlesim", "turtlesim_node")
    sl.node("turtle_draw", "turtle_draw")
    sl.node("turtle_draw", "turtle_sweep")

    return sl.launch_description()
