import socket
import json
import time
import random
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


def send(sock, command, delay=2):
    print(f"[Test] Sending:\n{json.dumps(command, indent=2)}")
    sock.sendall(json.dumps(command).encode('utf-8'))
    time.sleep(delay)


def create_cube_points(center_xy=(25, 25), size=20, min_z=5):
    """
    Generate the 8 corner points of a cube.
    The bottom face will be at Z = min_z.
    """
    cx, cy = center_xy
    s = size / 2
    cz = s + min_z  # so that cz - s = min_z

    corners = []
    for dx in [-s, s]:
        for dy in [-s, s]:
            for dz in [-s, s]:
                corners.append([cx + dx, cy + dy, cz + dz])
    return corners


def simulate_layering(points, vertical_step=2.5, grid_spacing=5.0):
    """
    Simulate the layering + gridding on the client side to visualize
    roughly what your server does.
    """
    import trimesh
    from shapely.geometry import Polygon, Point as ShapelyPoint

    points_np = np.array(points)
    hull = trimesh.convex.convex_hull(points_np)

    z_min = np.min(points_np[:, 2])
    z_max = np.max(points_np[:, 2])

    z_layers = np.arange(z_min, z_max + vertical_step, vertical_step)
    waypoints = []

    for z in z_layers:
        slice = hull.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
        if slice is None:
            continue

        # ✅ Use modern API
        path2D, T = slice.to_2D()

        polys = path2D.polygons_full
        if not polys:
            continue

        poly = polys[0]
        shapely_poly = Polygon(list(poly.exterior.coords))

        minx, miny, maxx, maxy = shapely_poly.bounds
        y_vals = np.arange(miny, maxy + grid_spacing, grid_spacing)

        for i, y in enumerate(y_vals):
            x_vals = np.arange(minx, maxx + grid_spacing, grid_spacing)
            row = []
            for x in x_vals:
                if shapely_poly.contains(ShapelyPoint(x, y)):
                    # ✅ Transform planar XY back to world XYZ using T
                    point_local = np.array([x, y, 0, 1])   # homogeneous
                    point_world = T @ point_local          # apply transform
                    row.append((point_world[0], point_world[1], point_world[2]))
            if i % 2 == 1:
                row.reverse()
            waypoints.extend(row)

    return waypoints

def plot_shape(points, waypoints):
    """
    Plot the random corner points and the layered waypoints.
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot corner points
    corner_points = np.array(points)
    ax.scatter(corner_points[:, 0], corner_points[:, 1], corner_points[:, 2],
               color='red', label='Corner Points', s=50)

    # Plot layered waypoints
    if waypoints:
        wps = np.array(waypoints)
        ax.plot(wps[:, 0], wps[:, 1], wps[:, 2],
                color='blue', marker='o', linestyle='-', label='Layered Waypoints')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Random 3D Shape + Sliced Waypoints')
    ax.legend()
    plt.show()


def main():
    host = "127.0.0.1"
    port = 65432

    try:
        print(f"[Test] Connecting to {host}:{port}...")

        cube_points = create_cube_points()
        # Simulate the layering for your own inspection
        layered_waypoints = simulate_layering(cube_points)

        # Show the plot
        plot_shape(cube_points, layered_waypoints)

        # Connect and send to server
        with socket.create_connection((host, port), timeout=5) as sock:
            print("[Test] Connected.")

            map_area_command = {
                "type": "map_area",
                "mission_id": "test_vertical_layering",
                "points": cube_points,
                "vertical_step": 2.5,
                "grid_spacing": 5.0,
                "priority": 3,
                "preempt": True
            }
            send(sock, map_area_command, delay=5)

            # Let mission run briefly
            time.sleep(10)

            # Land
            send(sock, {
                "type": "command",
                "command": "land",
                "id": 0
            })

            print("[Test] Vertical layering test completed.")

    except Exception as e:
        print(f"[Test] Error: {e}")


if __name__ == "__main__":
    main()
