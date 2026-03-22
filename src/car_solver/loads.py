"""Vehicle load case calculator.

Derives force magnitudes at each suspension pickup point from vehicle
geometry, mass distribution, and dynamic load factors defined in .env.
All forces in Newtons, distances in millimetres unless noted.
"""

from dataclasses import dataclass

from car_solver.config import Config

G = 9.81  # m/s^2


@dataclass(frozen=True)
class CornerWeights:
    """Static corner weights in Newtons."""
    front_left_n: float
    front_right_n: float
    rear_left_n: float
    rear_right_n: float

    @property
    def total_n(self) -> float:
        return self.front_left_n + self.front_right_n + self.rear_left_n + self.rear_right_n

    @property
    def front_total_n(self) -> float:
        return self.front_left_n + self.front_right_n

    @property
    def rear_total_n(self) -> float:
        return self.rear_left_n + self.rear_right_n


@dataclass(frozen=True)
class CornerLoads:
    """Dynamic loads at a single corner in Newtons."""
    vertical_n: float
    lateral_n: float
    longitudinal_n: float


@dataclass(frozen=True)
class TorsionLoad:
    """Torsion load case: equal and opposite vertical at front corners."""
    front_left_n: float
    front_right_n: float
    target_stiffness_nm_per_deg: float


@dataclass(frozen=True)
class BendingLoad:
    """Bending load case: distributed weight along sills."""
    total_vertical_n: float
    front_fraction: float
    rear_fraction: float


@dataclass(frozen=True)
class LoadCases:
    """All derived load cases for the vehicle."""
    static_corners: CornerWeights
    front_left_dynamic: CornerLoads
    front_right_dynamic: CornerLoads
    rear_left_dynamic: CornerLoads
    rear_right_dynamic: CornerLoads
    torsion: TorsionLoad
    bending: BendingLoad


def calculate_static_corners(cfg: Config) -> CornerWeights:
    """Calculate static corner weights assuming symmetric left/right."""
    total_weight_n = cfg.mass.total_kg * G
    front_total = total_weight_n * cfg.geometry.cog_front_bias
    rear_total = total_weight_n * (1 - cfg.geometry.cog_front_bias)
    return CornerWeights(
        front_left_n=front_total / 2,
        front_right_n=front_total / 2,
        rear_left_n=rear_total / 2,
        rear_right_n=rear_total / 2,
    )


def calculate_dynamic_corner(
    static_n: float,
    total_weight_n: float,
    vertical_g: float,
    lateral_g: float,
    braking_g: float,
    is_front: bool,
    lateral_front_bias: float,
    braking_front_bias: float,
) -> CornerLoads:
    """Calculate dynamic loads at a single corner.

    Vertical: static corner weight multiplied by vertical g factor.
    Lateral: total mass * lateral g, distributed by front/rear bias, halved per side.
    Longitudinal: total mass * braking g, distributed by front/rear bias, halved per side.
    """
    vertical = static_n * vertical_g

    lat_bias = lateral_front_bias if is_front else (1 - lateral_front_bias)
    lateral = total_weight_n * lateral_g * lat_bias / 2

    brake_bias = braking_front_bias if is_front else (1 - braking_front_bias)
    longitudinal = total_weight_n * braking_g * brake_bias / 2

    return CornerLoads(
        vertical_n=vertical,
        lateral_n=lateral,
        longitudinal_n=longitudinal,
    )


def calculate_load_cases(cfg: Config) -> LoadCases:
    """Derive all load cases from config."""
    static = calculate_static_corners(cfg)
    total_w = static.total_n

    dynamic_args = dict(
        total_weight_n=total_w,
        vertical_g=cfg.loads.vertical_g,
        lateral_g=cfg.loads.lateral_g,
        braking_g=cfg.loads.braking_g,
        lateral_front_bias=cfg.loads.lateral_front_bias,
        braking_front_bias=cfg.loads.braking_front_bias,
    )

    fl = calculate_dynamic_corner(static.front_left_n, is_front=True, **dynamic_args)
    fr = calculate_dynamic_corner(static.front_right_n, is_front=True, **dynamic_args)
    rl = calculate_dynamic_corner(static.rear_left_n, is_front=False, **dynamic_args)
    rr = calculate_dynamic_corner(static.rear_right_n, is_front=False, **dynamic_args)

    # Torsion: 1000N equal and opposite at front corners (per spec)
    torsion = TorsionLoad(
        front_left_n=1000.0,
        front_right_n=-1000.0,
        target_stiffness_nm_per_deg=cfg.loads.torsion_target_nm_per_deg,
    )

    bending = BendingLoad(
        total_vertical_n=total_w,
        front_fraction=cfg.geometry.cog_front_bias,
        rear_fraction=1 - cfg.geometry.cog_front_bias,
    )

    return LoadCases(
        static_corners=static,
        front_left_dynamic=fl,
        front_right_dynamic=fr,
        rear_left_dynamic=rl,
        rear_right_dynamic=rr,
        torsion=torsion,
        bending=bending,
    )


def print_load_summary(cases: LoadCases, cfg: Config) -> None:
    """Print a formatted summary of all load cases."""
    sc = cases.static_corners
    print("=" * 60)
    print("VEHICLE LOAD CASE SUMMARY")
    print("=" * 60)

    print(f"\nTotal vehicle mass: {cfg.mass.total_kg:.0f} kg")
    print(f"Total weight: {sc.total_n:.0f} N")
    print(f"Front bias: {cfg.geometry.cog_front_bias:.0%}")

    print("\n--- Static Corner Weights ---")
    print(f"  Front left:  {sc.front_left_n:>8.1f} N")
    print(f"  Front right: {sc.front_right_n:>8.1f} N")
    print(f"  Rear left:   {sc.rear_left_n:>8.1f} N")
    print(f"  Rear right:  {sc.rear_right_n:>8.1f} N")

    for name, corner in [
        ("Front Left", cases.front_left_dynamic),
        ("Front Right", cases.front_right_dynamic),
        ("Rear Left", cases.rear_left_dynamic),
        ("Rear Right", cases.rear_right_dynamic),
    ]:
        print(f"\n--- {name} Dynamic Loads ---")
        print(f"  Vertical:      {corner.vertical_n:>8.1f} N  ({cfg.loads.vertical_g:.1f}g)")
        print(f"  Lateral:       {corner.lateral_n:>8.1f} N  ({cfg.loads.lateral_g:.1f}g)")
        print(f"  Longitudinal:  {corner.longitudinal_n:>8.1f} N  ({cfg.loads.braking_g:.1f}g)")

    print("\n--- Torsion Load Case ---")
    print(f"  Front left:  {cases.torsion.front_left_n:>+8.1f} N")
    print(f"  Front right: {cases.torsion.front_right_n:>+8.1f} N")
    print(f"  Target stiffness: {cases.torsion.target_stiffness_nm_per_deg:.0f} Nm/deg")

    print("\n--- Bending Load Case ---")
    print(f"  Total vertical: {cases.bending.total_vertical_n:>8.1f} N")
    print(f"  Front fraction: {cases.bending.front_fraction:.0%}")
    print(f"  Rear fraction:  {cases.bending.rear_fraction:.0%}")

    # Safety-factored loads
    print("\n--- Safety-Factored Design Loads ---")
    sf = cfg.safety.general
    sf_susp = cfg.safety.suspension_pickup
    print(f"  General structure (SF={sf:.1f}):")
    print(f"    Max front corner vertical: {cases.front_left_dynamic.vertical_n * sf:>8.1f} N")
    print(f"    Max rear corner vertical:  {cases.rear_left_dynamic.vertical_n * sf:>8.1f} N")
    print(f"  Suspension pickups (SF={sf_susp:.1f}):")
    print(f"    Max front corner vertical: {cases.front_left_dynamic.vertical_n * sf_susp:>8.1f} N")
    print(f"    Max rear corner vertical:  {cases.rear_left_dynamic.vertical_n * sf_susp:>8.1f} N")

    print("\n" + "=" * 60)


if __name__ == "__main__":
    from car_solver.config import load_config

    cfg = load_config()
    cases = calculate_load_cases(cfg)
    print_load_summary(cases, cfg)
