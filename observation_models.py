import numpy as np

def odometry_observation_model():
    # TODO: Return identity matrix (3x3) if all state variables are observable
    C = np.identity(3)
    return C

def odometry_observation_model_2():
    # TODO: Return identity matrix (6x6) if all 6 state variables are observed
    C = np.identity(6)
    return C
