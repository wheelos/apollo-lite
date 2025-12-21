import re
import numpy as np
import matplotlib.pyplot as plt


class ReferenceLineVisualizer:
    POINT_PATTERN = re.compile(
        r"\{x:\s*(-?\d+\.\d+),\s*y:\s*(-?\d+\.\d+),\s*theta:\s*(-?\d+\.\d+),\s*kappa:\s*(-?\d+\.\d+),\s*dkappa:\s*(-?\d+\.\d+)\}"
    )

    def __init__(self, file_path):
        self.points = self.parse_points(file_path)

    def parse_points(self, file_path):
        points = []
        try:
            with open(file_path, 'r') as file:
                for line in file:
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
        except FileNotFoundError:
            print(f"Error: File not found at path: {file_path}")
            return []
        except Exception as e:
            print(f"Error reading file: {str(e)}")
            return []
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
            if i % 10 == 0:
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
    import sys

    if len(sys.argv) < 2:
        print("Usage: python script.py <log_file_path>")
        print("Example: python script.py data/log/create_reference_line_output.log.INFO.20250707_141929.151626")
        sys.exit(1)

    file_path = sys.argv[1]
    visualizer = ReferenceLineVisualizer(file_path)
    visualizer.plot()
