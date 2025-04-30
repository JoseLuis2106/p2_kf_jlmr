import numpy as np

def velocity_motion_model():
    def state_transition_matrix_A():
        # TODO: Define and return the 3x3 identity matrix A
        A = np.identity(3)
        return A

    def control_input_matrix_B(mu=np.zeros(3), delta_t=0):
        # TODO: Define B using current theta and timestep delta_t
        # B should apply linear and angular velocity to position and heading
        th = mu[2]
        B = np.array([[np.cos(th)*delta_t, 0      ],
                      [np.sin(th)*delta_t, 0      ],
                      [0,                  delta_t]])
        return B

    return state_transition_matrix_A, control_input_matrix_B
def velocity_motion_model_2():
    def A(delta_t=0):
        # TODO: Define and return the 6x6 constant velocity model transition matrix
        A = np.array([[1, 0, 0, delta_t, 0,       0      ],
                      [0, 1, 0, 0,       delta_t, 0      ],
                      [0, 0, 1, 0,       0,       delta_t],
                      [0, 0, 0, 1,       0,       0      ],
                      [0, 0, 0, 0,       1,       0      ],
                      [0, 0, 0, 0,       0,       1      ]])
        return A

    def B():
        # TODO: Return 6x2 zero matrix (no control input used in pure KF)
        B = np.zeros((6,2))
        return B

    return A, B
