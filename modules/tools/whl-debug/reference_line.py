import re
import numpy as np
import matplotlib.pyplot as plt


class ReferenceLineVisualizer:
    POINT_PATTERN = re.compile(
        r"\{x:\s*([-\d.]+),\s*y:\s*([-\d.]+),\s*theta:\s*([-\d.]+),\s*kappa:\s*([-\d.]+),\s*dkappa:\s*([-\d.]+)\}"
    )

    def __init__(self, log_lines):
        self.points = self.parse_points(log_lines)

    def parse_points(self, lines):
        points = []
        for line in lines:
            match = self.POINT_PATTERN.search(line)
            if match:
                x, y, heading, kappa, dkappa = map(float, match.groups())
                points.append(
                    {
                        "x": x,
                        "y": y,
                        "heading": heading,
                        "kappa": kappa,
                        "dkappa": dkappa,
                    }
                )
        return points

    def plot(self):
        if not self.points:
            print("No valid points found to plot.")
            return

        x = np.array([p["x"] for p in self.points])
        y = np.array([p["y"] for p in self.points])
        heading = np.array([p["heading"] for p in self.points])
        kappa = np.array([p["kappa"] for p in self.points])
        dkappa = np.array([p["dkappa"] for p in self.points])
        indices = np.arange(len(x))

        fig = plt.figure(figsize=(16, 10))
        fig.suptitle(
            "ReferenceLine Visualization (Position + Heading Arrows)", fontsize=16
        )

        # Main plot: trajectory and heading arrows
        ax_main = fig.add_axes([0.05, 0.15, 0.6, 0.75])
        ax_main.set_title("Reference Line (Trajectory + Heading)")
        ax_main.set_xlabel("x")
        ax_main.set_ylabel("y")
        ax_main.axis("equal")

        # Trajectory line and points (blue dots, fixed size)
        ax_main.plot(
            x, y, color="gray", linestyle="-", linewidth=1, label="Reference Line"
        )
        ax_main.scatter(
            x, y, color="blue", s=40, label="Points", edgecolors="k", zorder=5
        )

        # Arrows indicate heading direction
        u = np.cos(heading)
        v = np.sin(heading)
        arrow_length = 0.2
        ax_main.quiver(
            x,
            y,
            u,
            v,
            angles="xy",
            scale_units="xy",
            scale=1 / arrow_length,
            color="red",
            width=0.003,
            label="Heading",
            zorder=10,
        )

        # Annotate each point with heading
        for i in range(len(x)):
            ax_main.text(
                x[i],
                y[i],
                f"θ={heading[i]:.2f}",
                fontsize=8,
                ha="right",
                va="bottom",
                color="blue",
            )

        # Add 5% margin to axes to prevent arrows from being clipped
        x_margin = (x.max() - x.min()) * 0.1
        y_margin = (y.max() - y.min()) * 0.1
        ax_main.set_xlim(x.min() - x_margin, x.max() + x_margin)
        ax_main.set_ylim(y.min() - y_margin, y.max() + y_margin)

        # Right subplots: heading, kappa, dkappa vs index
        ax_heading = fig.add_axes([0.7, 0.68, 0.25, 0.22])
        ax_heading.plot(indices, heading, color="green")
        ax_heading.set_title("Heading (theta) vs Index")
        ax_heading.set_xlabel("Index")
        ax_heading.set_ylabel("Theta (rad)")

        ax_kappa = fig.add_axes([0.7, 0.4, 0.25, 0.22])
        ax_kappa.plot(indices, kappa, color="red")
        ax_kappa.set_title("Kappa vs Index")
        ax_kappa.set_xlabel("Index")
        ax_kappa.set_ylabel("Kappa")

        ax_dkappa = fig.add_axes([0.7, 0.12, 0.25, 0.22])
        ax_dkappa.plot(indices, dkappa, color="purple")
        ax_dkappa.set_title("Dkappa vs Index")
        ax_dkappa.set_xlabel("Index")
        ax_dkappa.set_ylabel("Dkappa")

        plt.show()


if __name__ == "__main__":
    example_logs = [
        "{x: 0.0, y: 0.0, theta: 0.0, kappa: 0.0, dkappa: 0.0}",
        "{x: 1.0, y: 1.0, theta: 0.1, kappa: 0.01, dkappa: 0.001}",
        "{x: 2.0, y: 1.8, theta: 0.15, kappa: 0.02, dkappa: 0.0015}",
        "{x: 3.0, y: 2.6, theta: 0.2, kappa: 0.03, dkappa: 0.002}",
        "{x: 4.0, y: 3.3, theta: 0.25, kappa: 0.04, dkappa: 0.0025}",
        # You can continue to add your data points...
    ]

    visualizer = ReferenceLineVisualizer(example_logs)
    visualizer.plot()
