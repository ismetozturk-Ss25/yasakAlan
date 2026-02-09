"""
run_100_tests.py -- Automated test runner for 100 random forbidden zone configurations

For each iteration:
1. Generate random forbidden zones
2. Update runner_main.c with new zones
3. Build and run test_runner.exe
4. Plot the result to figure/ directory
"""

import os
import sys
import random
import subprocess
import re
from dataclasses import dataclass

# ============================================================
# Zone generation (from forbidenZoneGenerate.py)
# ============================================================

@dataclass
class Rect:
    az_min: float
    az_max: float
    el_min: float
    el_max: float

def overlaps(a: Rect, b: Rect) -> bool:
    if a.az_max <= b.az_min or b.az_max <= a.az_min:
        return False
    if a.el_max <= b.el_min or b.el_max <= a.el_min:
        return False
    return True

def generate_zones(
    n: int = 16,
    az_range=(-170.0, 170.0),
    el_range=(-30.0, 60.0),
    az_size_range=(15.0, 45.0),
    el_size_range=(10.0, 35.0),
    max_tries: int = 50000,
    edge_touch_probability: float = 0.7,
) -> list:
    az_lo, az_hi = az_range
    el_lo, el_hi = el_range

    zones = []
    tries = 0

    def q(x): return round(x, 1)

    while len(zones) < n and tries < max_tries:
        tries += 1

        w = random.uniform(*az_size_range)
        h = random.uniform(*el_size_range)

        # AZ position - freely placed within envelope
        az_min = random.uniform(az_lo, az_hi - w)
        az_max = az_min + w

        # EL position - with probability, snap to top or bottom boundary
        if random.random() < edge_touch_probability:
            if random.random() < 0.5:
                # Touch bottom (el_min = envelope el_min)
                el_min = el_lo
                el_max = el_min + h
            else:
                # Touch top (el_max = envelope el_max)
                el_max = el_hi
                el_min = el_max - h
        else:
            # Free placement (not touching boundary)
            el_min = random.uniform(el_lo + 5, el_hi - h - 5)
            el_max = el_min + h

        # Clamp to envelope
        el_min = max(el_lo, el_min)
        el_max = min(el_hi, el_max)

        candidate = Rect(q(az_min), q(az_max), q(el_min), q(el_max))

        if any(overlaps(candidate, z) for z in zones):
            continue

        zones.append(candidate)

    return zones

def zones_to_c(zones: list, var_name: str = "in") -> str:
    lines = []
    for i, z in enumerate(zones):
        lines.append(f"/* Zone {i:02d} */")
        lines.append(f"{var_name}.forbidden[{i}].valid = 1;")
        lines.append(f"{var_name}.forbidden[{i}].az_min = {z.az_min:.1f}f; {var_name}.forbidden[{i}].el_min = {z.el_min:.1f}f;")
        lines.append(f"{var_name}.forbidden[{i}].az_max = {z.az_max:.1f}f; {var_name}.forbidden[{i}].el_max = {z.el_max:.1f}f;")
    return "\n".join(lines)

def generate_start_target(zones: list, envelope=(-180, 180, -30, 60)):
    """Generate random start and target that are not inside forbidden zones."""
    az_min, az_max, el_min, el_max = envelope

    def point_in_zones(az, el):
        for z in zones:
            if z.az_min < az < z.az_max and z.el_min < el < z.el_max:
                return True
        return False

    # Generate start
    for _ in range(1000):
        start_az = round(random.uniform(az_min + 10, az_max - 10), 2)
        start_el = round(random.uniform(el_min + 5, el_max - 5), 2)
        if not point_in_zones(start_az, start_el):
            break

    # Generate target (try to be far from start)
    for _ in range(1000):
        target_az = round(random.uniform(az_min + 10, az_max - 10), 2)
        target_el = round(random.uniform(el_min + 5, el_max - 5), 2)
        # Ensure some distance from start
        dist = abs(target_az - start_az) + abs(target_el - start_el)
        if not point_in_zones(target_az, target_el) and dist > 50:
            break

    return (start_az, start_el), (target_az, target_el)

# ============================================================
# Runner main.c update
# ============================================================

def update_runner_main(zones: list, start: tuple, target: tuple, filepath: str):
    """Update runner_main.c with new zones and start/target.

    Replaces everything between 'set_envelope(&in);' and 'return run_sim('
    with new start/target and zone definitions.
    """
    with open(filepath, 'r', encoding='utf-8') as f:
        content = f.read()

    # Build replacement block
    zone_code = zones_to_c(zones, "in")
    replacement = (
        f"    in.az_now = {start[0]:.2f}f;  in.el_now = {start[1]:.2f}f;\n"
        f"    in.az_cmd = {target[0]:.2f}f;  in.el_cmd = {target[1]:.2f}f;\n"
        f"\n"
        f"{zone_code}\n"
        f"\n"
    )

    # Replace everything between the two markers
    pattern = r'(set_envelope\(&in\);\s*\n).*?(\s*return run_sim\()'
    content = re.sub(pattern, r'\1' + replacement + r'\2', content, flags=re.DOTALL)

    with open(filepath, 'w', encoding='utf-8') as f:
        f.write(content)

# ============================================================
# Build and run
# ============================================================

def build_and_run(workdir: str) -> bool:
    """Build and run the test. Returns True if successful."""
    # Build
    result = subprocess.run(
        ['gcc', '-O2', '-Wall', '-Wextra', '-pedantic', '-std=c99', '-Isrc',
         'src/avoidance_preop.c', 'src/avoidance_opmode.c',
         'test/runner_main.c', '-o', 'test_runner.exe'],
        cwd=workdir,
        capture_output=True,
        text=True
    )
    if result.returncode != 0:
        print(f"  BUILD FAILED: {result.stderr}")
        return False

    # Run
    result = subprocess.run(
        ['./test_runner.exe'],
        cwd=workdir,
        capture_output=True,
        text=True
    )

    return True

# ============================================================
# Plot
# ============================================================

def run_plot(workdir: str, iteration: int):
    """Run plot_path.py to generate figure."""
    # Import and run plot_path
    result = subprocess.run(
        [sys.executable, 'plot_path.py', 'forbidden_zones_c.txt'],
        cwd=workdir,
        capture_output=True,
        text=True
    )
    if result.returncode != 0:
        print(f"  PLOT FAILED: {result.stderr}")

# ============================================================
# Main
# ============================================================

def main():
    workdir = os.path.dirname(os.path.abspath(__file__))
    runner_main_path = os.path.join(workdir, 'test', 'runner_main.c')

    # Ensure figure directory exists
    figure_dir = os.path.join(workdir, 'figure')
    os.makedirs(figure_dir, exist_ok=True)

    num_tests = 100
    passed = 0
    failed = 0

    print(f"Running {num_tests} random tests...")
    print("=" * 60)

    for i in range(1, num_tests + 1):
        print(f"\n[{i:3d}/{num_tests}] ", end="")

        # Generate random zones
        seed = random.randint(0, 999999999)
        random.seed(seed)

        try:
            zones = generate_zones(n=16)
        except RuntimeError as e:
            print(f"Zone generation failed: {e}")
            continue

        start, target = generate_start_target(zones)
        print(f"Seed={seed}, Start=({start[0]:.1f},{start[1]:.1f}), Target=({target[0]:.1f},{target[1]:.1f})")

        # Update runner_main.c
        update_runner_main(zones, start, target, runner_main_path)

        # Build and run
        if not build_and_run(workdir):
            failed += 1
            continue

        # Check result from output file
        output_file = os.path.join(workdir, 'forbidden_zones_c.txt')
        with open(output_file, 'r') as f:
            output = f.read()

        if 'RESULT: PASS' in output:
            print(f"  -> PASS", end="")
            passed += 1
        else:
            print(f"  -> FAIL/NO_PATH", end="")
            failed += 1

        # Extract step count
        m = re.search(r'(\d+) steps', output)
        if m:
            print(f" ({m.group(1)} steps)")
        else:
            print()

        # Plot
        run_plot(workdir, i)

    print("\n" + "=" * 60)
    print(f"SUMMARY: {passed} PASSED, {failed} FAILED out of {num_tests} tests")
    print(f"Figures saved to: {figure_dir}")

if __name__ == "__main__":
    main()
