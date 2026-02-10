import random
from dataclasses import dataclass

@dataclass
class Rect:
    az_min: float
    az_max: float
    el_min: float
    el_max: float

def overlaps(a: Rect, b: Rect) -> bool:
    """Axis-aligned rectangle overlap test (strict overlap). Touching edges is OK."""
    if a.az_max <= b.az_min or b.az_max <= a.az_min:
        return False
    if a.el_max <= b.el_min or b.el_max <= a.el_min:
        return False
    return True

def generate_zones(
    n: int = 32,
    az_range=(-170.0, 170.0),
    el_range=(-30.0, 60.0),
    az_size_range=(15.0, 45.0),
    el_size_range=(10.0, 35.0),
    seed: int = 42,
    max_tries: int = 20000,
    edge_touch_probability: float = 0.7,  # Probability that zone touches EL boundary
) -> list[Rect]:
    """
    Generate non-overlapping forbidden zones.

    With edge_touch_probability, zones are more likely to touch the EL envelope limits:
    - el_min = envelope el_min (bottom edge)
    - el_max = envelope el_max (top edge)
    """
    random.seed(seed)

    az_lo, az_hi = az_range
    el_lo, el_hi = el_range

    zones: list[Rect] = []
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
            # Touch an EL boundary
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
            el_min = random.uniform(el_lo + 5, el_hi - h - 5)  # Keep away from edges
            el_max = el_min + h

        # Clamp to envelope
        el_min = max(el_lo, el_min)
        el_max = min(el_hi, el_max)

        candidate = Rect(q(az_min), q(az_max), q(el_min), q(el_max))

        # Reject if overlaps any existing zone
        if any(overlaps(candidate, z) for z in zones):
            continue

        zones.append(candidate)

    if len(zones) < n:
        raise RuntimeError(
            f"Could only place {len(zones)}/{n} non-overlapping zones after {tries} tries. "
            "Try reducing size ranges or increasing envelope/max_tries."
        )

    return zones

def zones_to_c(zones: list[Rect], var_name: str = "in") -> str:
    lines = []
    for i, z in enumerate(zones):
        lines.append(f"/* Zone {i:02d} */")
        lines.append(f"{var_name}.forbidden[{i}].valid = 1;")
        lines.append(f"{var_name}.forbidden[{i}].az_min = {z.az_min:.1f}f; {var_name}.forbidden[{i}].el_min = {z.el_min:.1f}f;")
        lines.append(f"{var_name}.forbidden[{i}].az_max = {z.az_max:.1f}f; {var_name}.forbidden[{i}].el_max = {z.el_max:.1f}f;")
    return "\n".join(lines) + "\n"

def main():
    a = random.randint(0, 99999999999)
    #a = 757698269
    zones = generate_zones(
        n=32,
        az_range=(-170.0, 170.0),
        el_range=(-30.0, 60.0),
        az_size_range=(15.0, 45.0),
        el_size_range=(10.0, 35.0),
        seed=a,
        max_tries=50000,
        edge_touch_probability=0.7,  # 70% of zones touch EL boundary
    )

    # Count how many touch boundaries
    el_lo, el_hi = -30.0, 60.0
    bottom_touch = sum(1 for z in zones if abs(z.el_min - el_lo) < 0.2)
    top_touch = sum(1 for z in zones if abs(z.el_max - el_hi) < 0.2)

    c_text = zones_to_c(zones, var_name="in")

    out_path = "forbidden_zones_c.txt"
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(c_text)

    print(f"Wrote {len(zones)} zones to {out_path}")
    print(f"Zones touching bottom (EL=-30): {bottom_touch}")
    print(f"Zones touching top (EL=60): {top_touch}")
    print("\n--- Preview ---\n")
    print(c_text[:800])

if __name__ == "__main__":
    main()
