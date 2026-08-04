import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from GUI_MultiVMC_Config import (
    DEFAULT_START,
    DEFAULT_GOALS,
    VIRTUAL_COMPONENTS,
    STEPS_M,
    PUBLISH_PERIOD,
    LOOP_BACK,
    SYNCHRONIZED_ARRIVAL,
    MOVING_GOAL,
)

# TERMINAL 3 (ONLY AFTER running GUI_MultiVMC_Torque.py)
# python3 MultiVMC_MovingGoal_Pub.py

class MultiGoalPublisher(Node):
    """
    Publishes flattened multi-goal vector:
      [x1,y1,z1, x2,y2,z2, ..., xN,yN,zN]
    to /VC_ref_pos.
    """

    def __init__(self):
        super().__init__("goal_pos_multi_publisher")

        self.pub = self.create_publisher(Float64MultiArray, "/VC_ref_pos", 10)
        self.n_comp = len(VIRTUAL_COMPONENTS)

        self.starts = DEFAULT_START.copy()   # (N,3)
        self.ends = DEFAULT_GOALS.copy()     # (N,3)

        self.step_m = STEPS_M
        self.publish_period = PUBLISH_PERIOD
        self.loop_back = LOOP_BACK
        self.moving_goal = MOVING_GOAL

        self.idx = 0
        self.direction = 1

        if self.moving_goal:
            if SYNCHRONIZED_ARRIVAL:
                self.path = self.build_path_synchronized(self.starts, self.ends, self.step_m)  # (T,N,3)
            else:
                self.path = self.build_path_desynchronized(self.starts, self.ends, self.step_m)  # (T,N,3)

            mode = "SYNCHRONIZED" if SYNCHRONIZED_ARRIVAL else "DESYNCHRONIZED"
            self.get_logger().info(
                f"[MOVING_GOAL=True | {mode}] Publishing {self.n_comp} components on /VC_ref_pos, "
            )
        else:
            self.get_logger().info(
                f"[MOVING_GOAL=False] Publishing fixed DEFAULT_GOALS for {self.n_comp} components on /VC_ref_pos"
            )

        self.timer = self.create_timer(self.publish_period, self.timer_cb)

    @staticmethod
    def build_path_synchronized(starts: np.ndarray, ends: np.ndarray, step_m: float) -> np.ndarray:
        """
        Shared-alpha mode:
        - One global alpha timeline based on max component distance.
        - All components reach end at the same frame.
        """
        d = ends - starts                            # (N,3)
        d_norm = np.linalg.norm(d, axis=1)           # (N,)
        max_dist = float(np.max(d_norm))

        if max_dist < 1e-12:
            return starts[None, :, :]                # (1,N,3)

        n_steps = max(1, int(np.floor(max_dist / step_m)))
        alphas = np.linspace(0.0, 1.0, n_steps + 1)  # (T,)

        # path[t,i,:] = starts[i,:] + alpha[t]*(ends[i,:]-starts[i,:])
        path = starts[None, :, :] + alphas[:, None, None] * d[None, :, :]
        return path

    @staticmethod
    def build_path_desynchronized(starts: np.ndarray, ends: np.ndarray, step_m: float) -> np.ndarray:
        """
        Per-component mode:
        - Each component gets its own n_steps based on its own distance.
        - Components can finish at different times.
        - We pack into one (T,N,3) tensor by holding each component at final point
          after it finishes.
        """
        n_comp = starts.shape[0]
        comp_paths = []
        max_len = 1

        for i in range(n_comp):
            s = starts[i]                            # (3,)
            e = ends[i]                              # (3,)
            d = e - s
            dist = float(np.linalg.norm(d))

            if dist < 1e-12:
                p = s[None, :]                       # (1,3)
            else:
                n_steps_i = max(1, int(np.floor(dist / step_m)))
                alphas_i = np.linspace(0.0, 1.0, n_steps_i + 1)  # (Ti,)
                p = s[None, :] + alphas_i[:, None] * d[None, :]  # (Ti,3)

            comp_paths.append(p)
            max_len = max(max_len, p.shape[0])

        # Pack into (T,N,3), hold-last padding
        path = np.zeros((max_len, n_comp, 3), dtype=float)
        for i, p in enumerate(comp_paths):
            Ti = p.shape[0]
            path[:Ti, i, :] = p
            if Ti < max_len:
                path[Ti:, i, :] = p[-1, :]          # hold final point

        return path

    def timer_cb(self):
        if self.moving_goal:
            goals_now = self.path[self.idx]              # (N,3)
        else:
            goals_now = self.ends                        # (N,3) immediate fixed goal

        flat = goals_now.reshape(-1)                     # (3N,)

        msg = Float64MultiArray()
        msg.data = flat.tolist()
        self.pub.publish(msg)

        self.get_logger().info(
            f"Published x:{goals_now[:,0]}, y:{goals_now[:,1]}, z:{goals_now[:,2]}"
        )

        if not self.moving_goal:
            return  # keep publishing fixed goal, no path index update

        if not self.loop_back:
            if self.idx < len(self.path) - 1:
                self.idx += 1
        else:
            self.idx += self.direction
            if self.idx >= len(self.path):
                self.idx = len(self.path) - 2
                self.direction = -1
            elif self.idx < 0:
                self.idx = 1
                self.direction = 1


def main():
    rclpy.init()
    node = MultiGoalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()