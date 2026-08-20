0. Run GUI_MultiVMC_Torque.py and bash start_arm.sh
1. Run moving_gauss_test.py with k = 1.7 
2. Run moving_gauss_test.py with k = 16.7 

use moving_gauss.ipynb to generate plots 


Configuration:

    dict(
        component=VirtualComponent(
            # Gaussian spring produces max force when sigma = delta_x so when initializing on the robot it could produce very large forces
            # set stregnth to 0.0 to effectively disable the spring until we can move the reference in the GUI far from robot 
            # once the reference is moved we can increase the strength to a nonzero force
            spring=GaussianSpring(strength=0.0, sigma=0.06), 
            damper=LinearDamper(1.5),
            s_param=1.0,
            name="gauss_seg3",
        ),
        goal=gauss_y,# GUI OVERRIDES THIS GOAL --- IGNORE ---
    ),


DEFAULT_START = np.array(
    [_start_from_s_param(comp.s_param) for comp in VIRTUAL_COMPONENTS],
    dtype=float,
)  # (N,3)
