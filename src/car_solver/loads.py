"""Vehicle load case calculator.

Derives force magnitudes at each suspension pickup point from vehicle
geometry, mass distribution, and dynamic load factors defined in .env.
Also defines structured monocoque load cases that can be converted into
solver-ready force vectors by geometry-specific code.
"""

from dataclasses import dataclass
from typing import Literal

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


Axis = Literal["x", "y", "z"]
SupportSet = Literal["rear_face", "pickup_points"]


@dataclass(frozen=True)
class PointLoadSpec:
    """Point load applied to a named target node."""

    target: str
    axis: Axis
    magnitude_n: float


@dataclass(frozen=True)
class DistributedLoadSpec:
    """Total load distributed evenly over a named target node group."""

    target: str
    axis: Axis
    total_magnitude_n: float


LoadSpec = PointLoadSpec | DistributedLoadSpec


@dataclass(frozen=True)
class WeightedSubcase:
    """A solver subcase with one or more loads and an objective weight."""

    name: str
    loads: tuple[LoadSpec, ...]
    weight: float = 1.0


@dataclass(frozen=True)
class SolverLoadCaseDefinition:
    """Structured load-case definition decoupled from geometry-specific node ids."""

    name: str
    support_set: SupportSet
    subcases: tuple[WeightedSubcase, ...]
    description: str = ""


@dataclass(frozen=True)
class MonocoqueLoadDefinitions:
    """Structured monocoque load definitions for the current config."""

    torsion: SolverLoadCaseDefinition
    bending: SolverLoadCaseDefinition
    corner: SolverLoadCaseDefinition
    combined: SolverLoadCaseDefinition


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


def build_monocoque_load_definitions(cfg: Config) -> MonocoqueLoadDefinitions:
    """Build structured monocoque load definitions from config.

    These definitions are geometry-agnostic. Named targets such as
    ``front_left_pickup`` or ``left_sill_top`` are resolved into actual
    node ids by the monocoque geometry layer.
    """
    derived = calculate_load_cases(cfg)

    fl = derived.front_left_dynamic
    rl = derived.rear_left_dynamic

    front_total = fl.vertical_n + fl.lateral_n + fl.longitudinal_n
    rear_total = rl.vertical_n + rl.lateral_n + rl.longitudinal_n

    front_frac = 2.0 / 3.0  # spec split: 0.20 front vs 0.10 rear
    rear_frac = 1.0 / 3.0

    torsion = SolverLoadCaseDefinition(
        name="torsion",
        support_set="rear_face",
        description="Equal and opposite front-corner vertical loads with rear fixed.",
        subcases=(
            WeightedSubcase(
                name="torsion",
                loads=(
                    PointLoadSpec("front_left_pickup", "z", derived.torsion.front_left_n),
                    PointLoadSpec("front_right_pickup", "z", derived.torsion.front_right_n),
                ),
            ),
        ),
    )

    bending = SolverLoadCaseDefinition(
        name="bending",
        support_set="pickup_points",
        description="Vehicle weight distributed across both sill top edges.",
        subcases=(
            WeightedSubcase(
                name="bending",
                loads=(
                    DistributedLoadSpec(
                        "left_sill_top", "z", -derived.bending.total_vertical_n / 2
                    ),
                    DistributedLoadSpec(
                        "right_sill_top", "z", -derived.bending.total_vertical_n / 2
                    ),
                ),
            ),
        ),
    )

    corner = SolverLoadCaseDefinition(
        name="corner",
        support_set="pickup_points",
        description="Front and rear corner loads split into directional subcases.",
        subcases=(
            WeightedSubcase(
                name="front_right_vertical",
                loads=(PointLoadSpec("front_right_load", "z", -fl.vertical_n),),
                weight=front_frac * fl.vertical_n / front_total / 2,
            ),
            WeightedSubcase(
                name="front_right_lateral",
                loads=(PointLoadSpec("front_right_load", "y", fl.lateral_n),),
                weight=front_frac * fl.lateral_n / front_total / 2,
            ),
            WeightedSubcase(
                name="front_right_braking",
                loads=(PointLoadSpec("front_right_load", "x", fl.longitudinal_n),),
                weight=front_frac * fl.longitudinal_n / front_total / 2,
            ),
            WeightedSubcase(
                name="front_left_vertical",
                loads=(PointLoadSpec("front_left_load", "z", -fl.vertical_n),),
                weight=front_frac * fl.vertical_n / front_total / 2,
            ),
            WeightedSubcase(
                name="front_left_lateral",
                loads=(PointLoadSpec("front_left_load", "y", -fl.lateral_n),),
                weight=front_frac * fl.lateral_n / front_total / 2,
            ),
            WeightedSubcase(
                name="front_left_braking",
                loads=(PointLoadSpec("front_left_load", "x", fl.longitudinal_n),),
                weight=front_frac * fl.longitudinal_n / front_total / 2,
            ),
            WeightedSubcase(
                name="rear_right_vertical",
                loads=(PointLoadSpec("rear_right_load", "z", -rl.vertical_n),),
                weight=rear_frac * rl.vertical_n / rear_total / 2,
            ),
            WeightedSubcase(
                name="rear_right_lateral",
                loads=(PointLoadSpec("rear_right_load", "y", rl.lateral_n),),
                weight=rear_frac * rl.lateral_n / rear_total / 2,
            ),
            WeightedSubcase(
                name="rear_right_braking",
                loads=(PointLoadSpec("rear_right_load", "x", -rl.longitudinal_n),),
                weight=rear_frac * rl.longitudinal_n / rear_total / 2,
            ),
            WeightedSubcase(
                name="rear_left_vertical",
                loads=(PointLoadSpec("rear_left_load", "z", -rl.vertical_n),),
                weight=rear_frac * rl.vertical_n / rear_total / 2,
            ),
            WeightedSubcase(
                name="rear_left_lateral",
                loads=(PointLoadSpec("rear_left_load", "y", -rl.lateral_n),),
                weight=rear_frac * rl.lateral_n / rear_total / 2,
            ),
            WeightedSubcase(
                name="rear_left_braking",
                loads=(PointLoadSpec("rear_left_load", "x", -rl.longitudinal_n),),
                weight=rear_frac * rl.longitudinal_n / rear_total / 2,
            ),
        ),
    )

    combined = SolverLoadCaseDefinition(
        name="combined",
        support_set="rear_face",
        description="Exploratory combined weighted study using a shared rear-face support set.",
        subcases=(
            WeightedSubcase(
                name="torsion",
                loads=torsion.subcases[0].loads,
                weight=cfg.weights.torsion,
            ),
            WeightedSubcase(
                name="bending",
                loads=bending.subcases[0].loads,
                weight=cfg.weights.bending,
            ),
            WeightedSubcase(
                name="front_right_vertical",
                loads=(PointLoadSpec("front_right_load", "z", -fl.vertical_n),),
                weight=cfg.weights.front_corner * fl.vertical_n / front_total / 2,
            ),
            WeightedSubcase(
                name="front_right_lateral",
                loads=(PointLoadSpec("front_right_load", "y", fl.lateral_n),),
                weight=cfg.weights.front_corner * fl.lateral_n / front_total / 2,
            ),
            WeightedSubcase(
                name="front_right_braking",
                loads=(PointLoadSpec("front_right_load", "x", fl.longitudinal_n),),
                weight=cfg.weights.front_corner * fl.longitudinal_n / front_total / 2,
            ),
            WeightedSubcase(
                name="front_left_vertical",
                loads=(PointLoadSpec("front_left_load", "z", -fl.vertical_n),),
                weight=cfg.weights.front_corner * fl.vertical_n / front_total / 2,
            ),
            WeightedSubcase(
                name="front_left_lateral",
                loads=(PointLoadSpec("front_left_load", "y", -fl.lateral_n),),
                weight=cfg.weights.front_corner * fl.lateral_n / front_total / 2,
            ),
            WeightedSubcase(
                name="front_left_braking",
                loads=(PointLoadSpec("front_left_load", "x", fl.longitudinal_n),),
                weight=cfg.weights.front_corner * fl.longitudinal_n / front_total / 2,
            ),
            WeightedSubcase(
                name="rear_right_vertical",
                loads=(PointLoadSpec("rear_right_load", "z", -rl.vertical_n),),
                weight=cfg.weights.rear_corner * rl.vertical_n / rear_total / 2,
            ),
            WeightedSubcase(
                name="rear_right_lateral",
                loads=(PointLoadSpec("rear_right_load", "y", rl.lateral_n),),
                weight=cfg.weights.rear_corner * rl.lateral_n / rear_total / 2,
            ),
            WeightedSubcase(
                name="rear_right_braking",
                loads=(PointLoadSpec("rear_right_load", "x", -rl.longitudinal_n),),
                weight=cfg.weights.rear_corner * rl.longitudinal_n / rear_total / 2,
            ),
            WeightedSubcase(
                name="rear_left_vertical",
                loads=(PointLoadSpec("rear_left_load", "z", -rl.vertical_n),),
                weight=cfg.weights.rear_corner * rl.vertical_n / rear_total / 2,
            ),
            WeightedSubcase(
                name="rear_left_lateral",
                loads=(PointLoadSpec("rear_left_load", "y", -rl.lateral_n),),
                weight=cfg.weights.rear_corner * rl.lateral_n / rear_total / 2,
            ),
            WeightedSubcase(
                name="rear_left_braking",
                loads=(PointLoadSpec("rear_left_load", "x", -rl.longitudinal_n),),
                weight=cfg.weights.rear_corner * rl.longitudinal_n / rear_total / 2,
            ),
        ),
    )

    return MonocoqueLoadDefinitions(
        torsion=torsion,
        bending=bending,
        corner=corner,
        combined=combined,
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
