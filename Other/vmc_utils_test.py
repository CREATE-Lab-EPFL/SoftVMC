import numpy as np
import matplotlib.pyplot as plt


class LinearSpring:
    """Linear spring force: F = K * (target - current)"""

    def __init__(self, stiffness):
        self.stiffness = stiffness

    def _to_matrix(self, gain, dim):
        gain = np.asarray(gain)
        if gain.ndim == 0:
            return float(gain) * np.eye(dim)
        elif gain.ndim == 1:
            return np.diag(gain)
        else:
            return gain

    def compute_force(self, pos_current, pos_target):
        error = np.asarray(pos_target) - np.asarray(pos_current)
        return self._to_matrix(self.stiffness, len(error)) @ error


class GaussianSpring:
    """Gaussian repulsive spring for obstacle avoidance"""

    def __init__(self, strength, sigma):
        self.strength = strength
        self.sigma = sigma

    def compute_force(self, pos_current, pos_obstacle):
        delta = np.asarray(pos_current) - np.asarray(pos_obstacle)
        distance_sq = np.dot(delta, delta)
        decay = np.exp(-distance_sq / (2 * self.sigma**2))
        return self.strength * decay * delta


def main():
    # 1D sweep along x axis (y=z=0), so distance = |x|
    d = np.linspace(0.0, 0.20, 400)  # meters

    # -------- tune these --------
    linear_stiffness_list = [10, 25, 50, 100]           # N/m
    gaussian_strength_list = [20, 50, 100]              # amplitude
    gaussian_sigma_list = [0.01, 0.03, 0.05]            # meters
    # ----------------------------

    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    ax_mag, ax_x = axes

    # Linear spring (attractive to target at x=0): F = -K*x
    for K in linear_stiffness_list:
        spring = LinearSpring(stiffness=K)
        forces = []
        for x in d:
            f = spring.compute_force(
                pos_current=np.array([x, 0.0, 0.0]),
                pos_target=np.array([0.0, 0.0, 0.0]),
            )
            forces.append(f[0])  # Fx
        forces = np.array(forces)

        ax_mag.plot(d, np.abs(forces), label=f"Linear | K={K}")
        ax_x.plot(d, forces, label=f"Linear Fx | K={K}")

    # Gaussian spring (repulsive from obstacle at x=0): F = A*exp(-r^2/(2σ^2))*delta
    for A in gaussian_strength_list:
        for sigma in gaussian_sigma_list:
            spring = GaussianSpring(strength=A, sigma=sigma)
            forces = []
            for x in d:
                f = spring.compute_force(
                    pos_current=np.array([x, 0.0, 0.0]),
                    pos_obstacle=np.array([0.0, 0.0, 0.0]),
                )
                forces.append(f[0])  # Fx
            forces = np.array(forces)

            ax_mag.plot(d, np.abs(forces), "--", label=f"Gauss | A={A}, σ={sigma}")
            ax_x.plot(d, forces, "--", label=f"Gauss Fx | A={A}, σ={sigma}")

    ax_mag.set_title("Force magnitude vs distance")
    ax_mag.set_xlabel("Distance from target/obstacle [m]")
    ax_mag.set_ylabel("|F| [N]")
    ax_mag.grid(True, alpha=0.3)
    ax_mag.legend(fontsize=8, ncol=2)

    ax_x.set_title("Signed Fx vs distance (x >= 0)")
    ax_x.set_xlabel("x distance [m]")
    ax_x.set_ylabel("Fx [N]")
    ax_x.grid(True, alpha=0.3)
    ax_x.legend(fontsize=8, ncol=2)

    fig.suptitle("Spring comparison: Linear (solid) vs Gaussian (dashed)")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()