"""Load .env configuration into typed dataclasses."""

from dataclasses import dataclass
from pathlib import Path

from dotenv import dotenv_values


def _load_env() -> dict[str, str]:
    """Find and load .env file, falling back to .env.example."""
    root = Path(__file__).resolve().parents[2]
    env_file = root / ".env"
    if not env_file.exists():
        env_file = root / ".env.example"
    values = dotenv_values(env_file)
    return {k: v for k, v in values.items() if v is not None}


def _get(env: dict[str, str], key: str, cast: type = float):
    return cast(env[key])


@dataclass(frozen=True)
class MassConfig:
    monocoque_kg: float
    body_panels_kg: float
    engine_drivetrain_kg: float
    suspension_wheels_kg: float
    fuel_fluids_kg: float
    driver_kg: float

    @property
    def total_kg(self) -> float:
        return (
            self.monocoque_kg
            + self.body_panels_kg
            + self.engine_drivetrain_kg
            + self.suspension_wheels_kg
            + self.fuel_fluids_kg
            + self.driver_kg
        )


@dataclass(frozen=True)
class GeometryConfig:
    wheelbase_mm: float
    front_track_mm: float
    rear_track_mm: float
    overall_length_mm: float
    overall_width_mm: float
    overall_height_mm: float
    ground_clearance_mm: float
    cog_height_mm: float
    cog_front_bias: float


@dataclass(frozen=True)
class WheelConfig:
    front_wheel_diameter_in: float
    front_wheel_width_in: float
    front_tyre_width_mm: float
    front_tyre_aspect_ratio: float
    rear_wheel_diameter_in: float
    rear_wheel_width_in: float
    rear_tyre_width_mm: float
    rear_tyre_aspect_ratio: float

    @property
    def front_tyre_od_mm(self) -> float:
        sidewall = self.front_tyre_width_mm * self.front_tyre_aspect_ratio / 100
        return self.front_wheel_diameter_in * 25.4 + 2 * sidewall

    @property
    def rear_tyre_od_mm(self) -> float:
        sidewall = self.rear_tyre_width_mm * self.rear_tyre_aspect_ratio / 100
        return self.rear_wheel_diameter_in * 25.4 + 2 * sidewall


@dataclass(frozen=True)
class MaterialConfig:
    fibre_volume_fraction: float
    fibre_modulus_gpa: float
    resin_modulus_gpa: float
    orientation_efficiency: float
    composite_density_gcm3: float
    core_modulus_gpa: float
    core_thickness_mm: float
    carbon_skin_thickness_mm: float

    @property
    def composite_modulus_gpa(self) -> float:
        """Rule of mixtures with orientation efficiency factor."""
        vf = self.fibre_volume_fraction
        return self.orientation_efficiency * (
            vf * self.fibre_modulus_gpa + (1 - vf) * self.resin_modulus_gpa
        )

    @property
    def sandwich_bending_stiffness_n_mm(self) -> float:
        """Bending stiffness (EI per unit width) of a sandwich panel.

        Two carbon skins of thickness t separated by a printed core of
        thickness c. Uses parallel axis theorem: D = 2 * E_skin * t * (c/2 + t/2)^2
        plus the core contribution E_core * c^3 / 12.

        Returns stiffness in N.mm (E in MPa, dimensions in mm).
        """
        t = self.carbon_skin_thickness_mm
        c = self.core_thickness_mm
        e_skin = self.composite_modulus_gpa * 1000  # GPa to MPa
        e_core = self.core_modulus_gpa * 1000
        d_skin = (c + t) / 2  # skin centroid to neutral axis
        return 2 * e_skin * t * d_skin**2 + e_core * c**3 / 12


@dataclass(frozen=True)
class SafetyConfig:
    general: float
    suspension_pickup: float
    fatigue_critical: float


@dataclass(frozen=True)
class ObjectiveWeights:
    torsion: float
    bending: float
    front_corner: float
    rear_corner: float

    def __post_init__(self):
        total = self.torsion + self.bending + self.front_corner + self.rear_corner
        if abs(total - 1.0) > 0.01:
            raise ValueError(f"Objective weights must sum to 1.0, got {total:.3f}")


@dataclass(frozen=True)
class DynamicLoads:
    vertical_g: float
    lateral_g: float
    braking_g: float
    braking_front_bias: float
    lateral_front_bias: float
    torsion_target_nm_per_deg: float


@dataclass(frozen=True)
class SIMPConfig:
    penalty: float
    volume_fraction: float
    convergence_tolerance: float
    max_iterations: int
    filter_radius: float


@dataclass(frozen=True)
class ManufacturingConfig:
    min_wall_thickness_mm: float
    carbon_wall_thickness_mm: float
    max_overhang_angle_deg: float
    print_volume_x_mm: float
    print_volume_y_mm: float
    print_volume_z_mm: float
    min_lattice_cell_mm: float
    vent_hole_diameter_mm: float


@dataclass(frozen=True)
class Config:
    mass: MassConfig
    geometry: GeometryConfig
    wheels: WheelConfig
    material: MaterialConfig
    safety: SafetyConfig
    weights: ObjectiveWeights
    loads: DynamicLoads
    simp: SIMPConfig
    manufacturing: ManufacturingConfig


def load_config() -> Config:
    """Load configuration from .env file."""
    env = _load_env()

    def g(key, cast=float):
        return _get(env, key, cast)

    return Config(
        mass=MassConfig(
            monocoque_kg=g("MASS_MONOCOQUE"),
            body_panels_kg=g("MASS_BODY_PANELS"),
            engine_drivetrain_kg=g("MASS_ENGINE_DRIVETRAIN"),
            suspension_wheels_kg=g("MASS_SUSPENSION_WHEELS"),
            fuel_fluids_kg=g("MASS_FUEL_FLUIDS"),
            driver_kg=g("MASS_DRIVER"),
        ),
        geometry=GeometryConfig(
            wheelbase_mm=g("WHEELBASE_MM"),
            front_track_mm=g("FRONT_TRACK_MM"),
            rear_track_mm=g("REAR_TRACK_MM"),
            overall_length_mm=g("OVERALL_LENGTH_MM"),
            overall_width_mm=g("OVERALL_WIDTH_MM"),
            overall_height_mm=g("OVERALL_HEIGHT_MM"),
            ground_clearance_mm=g("GROUND_CLEARANCE_MM"),
            cog_height_mm=g("COG_HEIGHT_MM"),
            cog_front_bias=g("COG_FRONT_BIAS"),
        ),
        wheels=WheelConfig(
            front_wheel_diameter_in=g("FRONT_WHEEL_DIAMETER_IN"),
            front_wheel_width_in=g("FRONT_WHEEL_WIDTH_IN"),
            front_tyre_width_mm=g("FRONT_TYRE_WIDTH_MM"),
            front_tyre_aspect_ratio=g("FRONT_TYRE_ASPECT_RATIO"),
            rear_wheel_diameter_in=g("REAR_WHEEL_DIAMETER_IN"),
            rear_wheel_width_in=g("REAR_WHEEL_WIDTH_IN"),
            rear_tyre_width_mm=g("REAR_TYRE_WIDTH_MM"),
            rear_tyre_aspect_ratio=g("REAR_TYRE_ASPECT_RATIO"),
        ),
        material=MaterialConfig(
            fibre_volume_fraction=g("FIBRE_VOLUME_FRACTION"),
            fibre_modulus_gpa=g("FIBRE_MODULUS_GPA"),
            resin_modulus_gpa=g("RESIN_MODULUS_GPA"),
            orientation_efficiency=g("ORIENTATION_EFFICIENCY"),
            composite_density_gcm3=g("COMPOSITE_DENSITY_GCM3"),
            core_modulus_gpa=g("CORE_MODULUS_GPA"),
            core_thickness_mm=g("CORE_THICKNESS_MM"),
            carbon_skin_thickness_mm=g("CARBON_SKIN_THICKNESS_MM"),
        ),
        safety=SafetyConfig(
            general=g("SF_GENERAL"),
            suspension_pickup=g("SF_SUSPENSION_PICKUP"),
            fatigue_critical=g("SF_FATIGUE_CRITICAL"),
        ),
        weights=ObjectiveWeights(
            torsion=g("WEIGHT_TORSION"),
            bending=g("WEIGHT_BENDING"),
            front_corner=g("WEIGHT_FRONT_CORNER"),
            rear_corner=g("WEIGHT_REAR_CORNER"),
        ),
        loads=DynamicLoads(
            vertical_g=g("VERTICAL_G_FACTOR"),
            lateral_g=g("LATERAL_G_FACTOR"),
            braking_g=g("BRAKING_G_FACTOR"),
            braking_front_bias=g("BRAKING_FRONT_BIAS"),
            lateral_front_bias=g("LATERAL_FRONT_BIAS"),
            torsion_target_nm_per_deg=g("TORSION_TARGET_NM_PER_DEG"),
        ),
        simp=SIMPConfig(
            penalty=g("SIMP_PENALTY"),
            volume_fraction=g("SIMP_VOLUME_FRACTION"),
            convergence_tolerance=g("SIMP_CONVERGENCE_TOLERANCE"),
            max_iterations=g("SIMP_MAX_ITERATIONS", int),
            filter_radius=g("SIMP_FILTER_RADIUS"),
        ),
        manufacturing=ManufacturingConfig(
            min_wall_thickness_mm=g("MIN_WALL_THICKNESS_MM"),
            carbon_wall_thickness_mm=g("CARBON_WALL_THICKNESS_MM"),
            max_overhang_angle_deg=g("MAX_OVERHANG_ANGLE_DEG"),
            print_volume_x_mm=g("PRINT_VOLUME_X_MM"),
            print_volume_y_mm=g("PRINT_VOLUME_Y_MM"),
            print_volume_z_mm=g("PRINT_VOLUME_Z_MM"),
            min_lattice_cell_mm=g("MIN_LATTICE_CELL_MM"),
            vent_hole_diameter_mm=g("VENT_HOLE_DIAMETER_MM"),
        ),
    )
