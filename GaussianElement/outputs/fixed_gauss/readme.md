Run the GUI interface:
0. Run GUI_MultiVMC_Torque.py and bash start_arm.sh and GUI_MultiVMC.py
1. put gauss_seg1 and gauss_seg2 to x = -17cm
2. set lin_seg3 = 33 (stiffness), gauss_seg1 = 240 , gauss_seg2 = 200 (strength)
3. Select lin_seg3 and move it to x = -50cm 
4. Once reached steady state, set gauss_seg1 and gauss_seg2 = 0 (strength)


Configuration:
dict(
        component=VirtualComponent(
            # Gaussian spring produces max force when sigma = delta_x so when initializing on the robot it could produce very large forces
            # set stregnth to 0.0 to effectively disable the spring until we can move the reference in the GUI far from robot 
            # once the reference is moved we can increase the strength to a nonzero force Typically stregnth = 140 is quite good
            # the max force produced by the gauss spring is when delta_x = sigma 
            spring=GaussianSpring(strength=0.0, sigma=0.06), 
            damper=LinearDamper(1.5),
            s_param=1.0 / 3.0,
            name="gauss_seg1",
        ),
        goal=gauss_y,# GUI OVERRIDES THIS GOAL --- IGNORE ---
    ),
    dict(
        component=VirtualComponent(
            # Gaussian spring produces max force when sigma = delta_x so when initializing on the robot it could produce very large forces
            # set stregnth to 0.0 to effectively disable the spring until we can move the reference in the GUI far from robot 
            # once the reference is moved we can increase the strength to a nonzero force
            spring=GaussianSpring(strength=0.0, sigma=0.06), 
            damper=LinearDamper(1.5),
            s_param=2.0 / 3.0,
            name="gauss_seg2",
        ),
        goal=gauss_y,# GUI OVERRIDES THIS GOAL --- IGNORE ---
    ),

    DEFAULT_START = np.array([
    [0.0, 0.0, 0.435],   # lin_seg3  (s=1.0)
    [0.0, 0.0, 0.145+0.0725],   # gauss_seg1 (s=1/3)
    [0.0, 0.0, 0.145+0.0725],   # gauss_seg2 (s=2/3)
], dtype=float)  # (N,3)


