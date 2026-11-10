#!/usr/bin/env python3
"""Generate obstacle variants of simulation/worlds/simple_goal_demo.sdf.

Each variant is a copy of the base world with a set of static, dark obstacles
inserted. The world name stays `simple_goal_demo`, so launch files, PX4 and the
camera topics keep working; only the file name differs.

The UGV is not copied: the base world includes model://simple_ugv, so the
variants pick up vehicle changes automatically. Never hand-edit a variant;
change the base world or this script and regenerate with --all.

Obstacles are dark on purpose: the mapper marks a grid cell occupied only when
projected camera pixels fall below `obstacle_luminance_threshold` (default 32).

Usage (from anywhere):
  python3 generate_obstacle_worlds.py --world <repo>/simulation/worlds/simple_goal_demo.sdf \
      --out-dir <repo>/simulation/worlds --all [--preview layouts.png]
  python3 generate_obstacle_worlds.py --world ... --layout dense
"""
import argparse
import math
import sys
from pathlib import Path

START = (-4.0, 0.0)  # UGV spawn in the base world
GOAL = (4.0, 0.0)  # goal marker in the base world
HEIGHT = 0.3  # keep low: the mapper projects onto flat ground, so tall objects smear
COLOR = "0.05 0.05 0.05 1"  # well under the 32/255 luminance threshold

# Obstacles must stay inside the area the drone camera surely covers (UAV at the
# origin, ~4.5 m up, ~100 deg horizontal FOV, 4:3 image => about 10.7 x 8.0 m).
X_LIMIT = 3.2
Y_LIMIT = 3.0
MIN_START_GOAL_CLEARANCE = 1.5  # edge-to-centre, metres
MIN_PASSABLE_GAP = 1.6  # robot radius 0.5 + 0.05 clearance, inflated, plus cell size

# ("box", x, y, size_x, size_y) or ("cyl", x, y, radius)
LAYOUTS = {
    "sparse": {
        "desc": "Three separated obstacles; one sits on the straight start-goal line.",
        "stress": False,
        "shapes": [
            ("box", -1.5, 0.0, 0.8, 0.8),
            ("cyl", 1.0, 1.2, 0.4),
            ("box", 2.0, -1.5, 0.6, 1.0),
        ],
    },
    "dense": {
        "desc": "Seven obstacles in a staggered field with 2.0 m gaps; forces a weaving route.",
        "stress": False,
        "shapes": [
            ("cyl", -2.6, 1.3, 0.3),
            ("cyl", -2.6, -1.3, 0.3),
            ("box", 0.0, 0.0, 0.6, 0.6),
            ("cyl", 0.0, 2.6, 0.3),
            ("cyl", 0.0, -2.6, 0.3),
            ("cyl", 2.6, 1.3, 0.3),
            ("cyl", 2.6, -1.3, 0.3),
        ],
    },
    "corridor": {
        "desc": "A wall across the route with a 2.0 m opening on the line; route should pass through it.",
        "stress": False,
        "shapes": [
            ("box", 0.0, -2.0, 0.4, 2.0),
            ("box", 0.0, 2.0, 0.4, 2.0),
        ],
    },
    "narrow": {
        "desc": "Same wall with a 1.2 m opening, only 0.1 m wider than the planner's 1.1 m "
                "clearance diameter. Stress test: with 0.1 m cells the route threads the gap "
                "slowly; with 0.25 m cells it is blocked. Never a collision.",
        "stress": True,
        "shapes": [
            ("box", 0.0, -1.8, 0.4, 2.4),
            ("box", 0.0, 1.8, 0.4, 2.4),
        ],
    },
}


def extents(shape):
    kind, x, y, *dims = shape
    if kind == "box":
        return x - dims[0] / 2, x + dims[0] / 2, y - dims[1] / 2, y + dims[1] / 2
    r = dims[0]
    return x - r, x + r, y - r, y + r


def dist_point_to_shape(px, py, shape):
    """Distance from a point to the edge of a shape (0 if inside)."""
    kind, x, y, *dims = shape
    if kind == "box":
        hx, hy = dims[0] / 2, dims[1] / 2
        dx = max(abs(px - x) - hx, 0.0)
        dy = max(abs(py - y) - hy, 0.0)
        return math.hypot(dx, dy)
    return max(math.hypot(px - x, py - y) - dims[0], 0.0)


def gap_between(a, b):
    """Edge-to-edge distance between two shapes (0 if touching or overlapping)."""
    if a[0] == "box" and b[0] == "box":
        dx = max(abs(a[1] - b[1]) - (a[3] + b[3]) / 2, 0.0)
        dy = max(abs(a[2] - b[2]) - (a[4] + b[4]) / 2, 0.0)
        return math.hypot(dx, dy)
    if a[0] == "cyl" and b[0] == "cyl":
        return max(math.hypot(a[1] - b[1], a[2] - b[2]) - a[3] - b[3], 0.0)
    cyl, box = (a, b) if a[0] == "cyl" else (b, a)
    return max(dist_point_to_shape(cyl[1], cyl[2], box) - cyl[3], 0.0)


def check_layout(name, shapes):
    """Return (problems, min_gap, min_start_goal_clearance)."""
    problems = []
    for i, s in enumerate(shapes, 1):
        x0, x1, y0, y1 = extents(s)
        if x0 < -X_LIMIT or x1 > X_LIMIT or y0 < -Y_LIMIT or y1 > Y_LIMIT:
            problems.append(f"obstacle_{i} leaves the safe camera area")
    clearance = min(
        dist_point_to_shape(px, py, s) for s in shapes for (px, py) in (START, GOAL)
    )
    if clearance < MIN_START_GOAL_CLEARANCE:
        problems.append(f"obstacle only {clearance:.2f} m from start or goal")
    gaps = [gap_between(a, b) for i, a in enumerate(shapes) for b in shapes[i + 1:]]
    min_gap = min(gaps) if gaps else float("inf")
    if not LAYOUTS[name]["stress"] and min_gap < MIN_PASSABLE_GAP:
        problems.append(f"smallest gap {min_gap:.2f} m is below {MIN_PASSABLE_GAP} m")
    return problems, min_gap, clearance


def model_xml(index, shape):
    kind, x, y, *dims = shape
    if kind == "box":
        geom = f"<box><size>{dims[0]} {dims[1]} {HEIGHT}</size></box>"
    else:
        geom = f"<cylinder><radius>{dims[0]}</radius><length>{HEIGHT}</length></cylinder>"
    return (
        f'    <model name="obstacle_{index}">\n'
        f"      <static>true</static>\n"
        f"      <pose>{x} {y} {HEIGHT / 2:.3f} 0 0 0</pose>\n"
        f'      <link name="link">\n'
        f'        <collision name="collision"><geometry>{geom}</geometry></collision>\n'
        f'        <visual name="visual">\n'
        f"          <geometry>{geom}</geometry>\n"
        f"          <material><ambient>{COLOR}</ambient><diffuse>{COLOR}</diffuse></material>\n"
        f"        </visual>\n"
        f"      </link>\n"
        f"    </model>\n"
    )


def build_world(base_text, name, shapes):
    if "generate_obstacle_worlds" in base_text:
        sys.exit("Base world already contains generated obstacles; pass the original file.")
    block = (
        f"    <!-- generate_obstacle_worlds.py: layout '{name}'. Obstacles are dark so the\n"
        f"         camera luminance mapper marks them occupied. -->\n"
        + "".join(model_xml(i, s) for i, s in enumerate(shapes, 1))
        + "\n"
    )
    marker = '<light name="sun"'
    pos = base_text.find(marker)
    if pos == -1:
        pos = base_text.rfind("</world>")
        if pos == -1:
            sys.exit("Could not find an insertion point in the base world.")
    pos = base_text.rfind("\n", 0, pos) + 1  # start of that line
    return base_text[:pos] + block + base_text[pos:]


def save_preview(path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle

    alt, hfov = 4.5, 1.74
    half_long = alt * math.tan(hfov / 2)
    half_short = half_long * 0.75
    fig, axes = plt.subplots(2, 2, figsize=(11, 8.5))
    for ax, (name, spec) in zip(axes.flat, LAYOUTS.items()):
        for hx, hy in ((half_long, half_short), (half_short, half_long)):
            ax.add_patch(Rectangle((-hx, -hy), 2 * hx, 2 * hy, fill=False, ls="--",
                                   ec="#888", lw=1))
        ax.add_patch(Rectangle((-X_LIMIT, -Y_LIMIT), 2 * X_LIMIT, 2 * Y_LIMIT,
                               fill=False, ec="#2a7", lw=1.2))
        for s in spec["shapes"]:
            if s[0] == "box":
                ax.add_patch(Rectangle((s[1] - s[3] / 2, s[2] - s[4] / 2), s[3], s[4],
                                       fc="#222", ec="k"))
            else:
                ax.add_patch(Circle((s[1], s[2]), s[3], fc="#222", ec="k"))
        ax.add_patch(Circle(START, 0.5, fc="#4a90d9", ec="k", alpha=0.8))
        ax.add_patch(Circle(GOAL, 0.8, fc="#f28c28", ec="k", alpha=0.6))
        ax.plot([START[0], GOAL[0]], [0, 0], ":", c="#555")
        ax.set_aspect("equal")
        ax.set_xlim(-6, 6)
        ax.set_ylim(-6, 6)
        probs, gap, clr = check_layout(name, spec["shapes"])
        ax.set_title(f"{name}  (min gap {gap:.1f} m)" if gap != float("inf") else name)
        ax.grid(alpha=0.2)
    fig.suptitle("Obstacle layouts, top view (map frame, metres). Blue = UGV start, "
                 "orange = goal,\ngreen = obstacle limit, dashed grey = approx. camera "
                 "footprint (both axis orientations)", fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=130)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--world", required=True, type=Path, help="Path to the base simple_goal_demo.sdf")
    ap.add_argument("--out-dir", required=True, type=Path)
    ap.add_argument("--layout", choices=sorted(LAYOUTS))
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--preview", type=Path, help="Write a top-view PNG of all layouts")
    args = ap.parse_args()
    if not args.layout and not args.all and not args.preview:
        ap.error("pass --layout NAME, --all, or --preview")

    names = sorted(LAYOUTS) if args.all else ([args.layout] if args.layout else [])
    base = args.world.read_text()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    failed = False
    for name in names:
        shapes = LAYOUTS[name]["shapes"]
        problems, gap, clearance = check_layout(name, shapes)
        out = args.out_dir / f"simple_goal_demo_{name}.sdf"
        out.write_text(build_world(base, name, shapes))
        status = "ok" if not problems else "PROBLEM: " + "; ".join(problems)
        print(f"{name:9s} {len(shapes)} obstacles  min gap {gap:5.2f} m  "
              f"start/goal clearance {clearance:4.2f} m  -> {out.name}  [{status}]")
        failed |= bool(problems)
    if args.preview:
        save_preview(args.preview)
        print(f"preview -> {args.preview}")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
