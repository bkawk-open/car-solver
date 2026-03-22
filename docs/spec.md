# Project Blacktub -- Build Specification
### America's Most Beautiful Roadster Contender
*Compiled March 2026*

---

## Vision Statement

A competition roadster built entirely from first principles. Every system -- computational, mechanical, and structural -- derived, fabricated, or written rather than bought off the shelf. The manufacturing process is the aesthetic. The structural logic is the visual language. A 1930s roadster silhouette containing an entirely novel engineering proposition.

Target competition: America's Most Beautiful Roadster, Grand National Roadster Show, Pomona Fairplex, California.

Timeline: 5 years to debut in finished form.

---

## Design Concept

The car references the pre-1937 open roadster form required for AMBR eligibility. No side windows. Long, low, dramatically flared rear arches. A continuous sculpted body shell with integrated louvre detailing. The render direction established in early concept work defines the proportion and attitude.

The defining visual idea: a forged carbon outer skin thin enough that the internal printed lattice structure reads through it. The geometry visible beneath the surface is not decoration. It is the mathematically optimised structural solution to the car's load cases, made visible. Every branch and density variation in the lattice is there because the topology optimisation algorithm determined it needed to be there.

Form and function are not balanced. They are identical.

---

## AMBR Eligibility Notes

- Vehicle must resemble a 1937 or earlier US-built roadster. Designer roadsters that meet this aesthetic qualify.
- No side windows permitted.
- Vehicle must start, stop, move forward and backward, and turn left and right under its own power before judging begins.
- Vehicle must make its debut in finished form at GNRS. No finished-state photography or video may be published before the unveiling.
- Judging covers: originality, style, design, craftsmanship, engineering, and functionality.
- Special class awards for: engine, interior, paint, detail, undercarriage, engineering, and display.
- The engineering award category is a primary target alongside the overall AMBR title.

---

## Guiding Principles

Every decision in this build is evaluated against these principles in order:

1. Can it be derived, designed, or fabricated rather than bought?
2. Does the manufacturing process contribute to the visual and engineering story?
3. Is the constraint set feeding the generative design system rather than fighting it?
4. Will a judge who examines this for ten minutes find more to look at than one who spent thirty seconds?

---

## System Overview

| System | Approach | Key Decision |
|---|---|---|
| Generative design software | Custom written Python | SIMP topology optimisation with process-specific constraints |
| Large format 3D printer | Built around bought print head | CoreXY, 1.2x1.2m bed minimum |
| Forged carbon injection machine | Fabricated from scratch | Screw drive, heated barrel, PID controlled |
| Monocoque tooling | Printed sacrificial outer + permanent inner lattice | Distributed vent hole system |
| Monocoque structure | Forged carbon over printed lattice core | Carbon sandwich, visible internal geometry |
| Body panels | Large format polymer print + forged carbon | Polymer for non-structural skin, LPBF metal for door structure |
| Chassis subframes | Chromoly TIG welded | Front and rear bolting to monocoque hard points |
| Drivetrain | Motorcycle engine donor, TBD | Target 300-400bhp, light weight priority |
| Autoclave | Fabricated pressure vessel | For cosmetic carbon components requiring surface quality |

---

## Stage 1 -- Software and Design
### Year 1

This stage produces no physical parts. It produces the computational foundation everything else is built on and a fully resolved design that does not need to change once fabrication begins.

### 1.1 Topology Optimisation Software

**Language:** Python
**Core algorithm:** SIMP (Solid Isotropic Material with Penalisation)
**Reference implementation:** topy as a learning reference, diverged from immediately to encode process-specific constraints
**FEA solver:** Calculix wrapped via Python, potentially replaced with custom sparse solver if performance demands it
**Visualisation:** PyVista with live updating render window, background solver thread pushing density field snapshots

**Development sequence:**

Start with a 2D implementation. Same algorithm, same constraint logic, but planar. Validates the mathematics rapidly. Cases with known correct answers -- a beam in bending, a plate with a hole -- confirm the solver before adding complexity.

Extend to 3D voxel representation once 2D is producing sensible results.

Add load cases incrementally. Torsion first. Then bending. Then corner loads.

Add manufacturing constraints one at a time, observing the geometric change each constraint produces. Document each transition.

**Voxel data structure per element:**

- Density value (0.0 to 1.0)
- Designable flag (can the optimiser change this element)
- Obstacle flag (permanently void -- drivetrain tunnel, cockpit, wheel arches)
- Preserve flag (permanently solid -- suspension pickup nodes, bulkheads)
- Vent-connected flag (does this void element connect to the outer vent network)
- Load application flag
- Boundary condition flag

**Manufacturing constraints to encode:**

- Minimum wall thickness (function of print nozzle diameter and filament choice)
- Flow path continuity (every void region must connect to at least one vent path to the outer surface)
- Print orientation (no overhangs beyond threshold angle without support geometry)
- Print volume partitioning (geometry automatically sectioned at designable split lines when exceeding build volume)
- Minimum lattice cell size (function of injection compound flow characteristics)

**Objective function:**

Weighted sum of compliances across all load cases. Initial weights:

- Torsional stiffness: 0.50
- Bending stiffness: 0.20
- Front corner loads (vertical + lateral): 0.20
- Rear corner loads: 0.10

Weights to be refined as vehicle dynamics load magnitudes are calculated more precisely.

**Output pipeline:**

Optimised density field exported as mesh via meshio. Threshold applied to convert to binary solid/void. Mesh cleaned and exported as STL for slicer input. Full pipeline from load case definition to print file is custom software throughout.

**Live visualisation targets:**

- Density field rendered as 3D volume, opacity mapped to density
- Von Mises stress field overlaid on surviving material, blue to red
- Constraint violation highlighting in warning colour
- Convergence metric plotted as real-time curve alongside 3D view
- Volume fraction slider adjustable during optimisation run
- Penalty factor slider adjustable during run
- Full iteration history captured as frames, rendered as timelapse video per component

**Documentation output:**

Convergence curves for each major component. Prediction versus measured result correlation tables. Constraint contribution visualisations showing geometry change per constraint added. Git repository with meaningful commit history telling the software development story.

### 1.2 Load Case Definition

**Vehicle mass estimate (initial):**

| Component | Estimate |
|---|---|
| Carbon monocoque tub | 60 kg |
| Body panels | 20 kg |
| Engine and drivetrain | 80 kg |
| Suspension and wheels | 60 kg |
| Fuel and fluids | 20 kg |
| Driver | 80 kg |
| **Total** | **320 kg** |

To be refined through iteration as design develops.

**Primary load cases:**

Torsional: equal and opposite 1000N vertical loads at front left and front right pickup points, rear fixed. Target torsional stiffness 10,000 Nm/degree minimum.

Bending: distributed load along sill representing vehicle weight at 1g, fixed at all four pickup points.

Front corner vertical: 80kg static corner weight times 3g dynamic factor = 2350N per front corner.

Front corner lateral: total vehicle mass times 1.5g lateral acceleration, distributed 60% front, gives approximately 2800N lateral at each front corner.

Braking longitudinal: 1.2g deceleration, 70% front bias, approximately 1300N longitudinal per front corner.

Rear corner loads: equivalent calculation at rear with appropriate distribution.

Engine mount torque reaction: to be calculated once engine selection is finalised.

Scuttle local: separate local load case at windscreen surround, magnitude TBD from scuttle shake targets.

**Safety factors:**

- General structure: 2.0 on ultimate load
- Suspension pickup regions: 3.0 to 4.0
- Fatigue critical areas: 3.5 minimum

### 1.3 Material Property Calculations

**Forged carbon composite -- rule of mixtures approximation:**

- Target fibre volume fraction: 45% (achievable with injection process and distributed venting)
- Orientation efficiency factor for random chopped fibre: 0.375
- Carbon fibre modulus (T300 standard): 230 GPa
- Epoxy resin modulus: 3.5 GPa
- Estimated composite tensile modulus: ~39 GPa
- Estimated tensile strength: 300-350 MPa
- Estimated density: 1.49 g/cm3
- Weight saving versus aluminium equivalent volume: ~45%

**Printed core material (carbon filled nylon or PEI):**

- Tensile modulus: 3-8 GPa depending on fill percentage
- Role: maintain skin separation, carry inter-skin shear loads
- Not required to match carbon skin stiffness

**Sandwich panel bending stiffness:**

Bending stiffness scales with cube of skin separation distance. A 10mm printed core between 2mm carbon skins produces dramatically higher bending stiffness than a 4mm solid carbon panel at equivalent or lower weight.

**Physical validation:**

Test coupons from first injection trials to be sent to a university or commercial materials testing lab. Full mechanical property test set approximately £200-400. Measured properties fed back into FEA material definitions. Prediction versus measurement correlation documented.

### 1.4 Design Resolution

The full car design must be completely resolved in CAD before any fabrication begins. Every panel, shut line, proportion, and detail must be intentional and final.

Key design decisions to resolve in Stage 1:

- Final body proportions derived from AMBR eligibility constraints and the established render direction
- Monocoque tub geometry defining the primary structural envelope
- Suspension geometry and pickup point locations (drives all structural load cases)
- Drivetrain packaging envelope (defines obstacle geometry for optimisation)
- Driver ergonomics and visibility envelope
- Panel split lines designed around print volume constraints and natural shut lines
- Lattice geometry family selection -- gyroid, Voronoi, geodesic, or custom -- and how it varies across structural zones
- Vent hole pattern density and diameter (informed by injection compound flow testing in Stage 2)

**Design principle:**

Where forged carbon is used without the printed core system, surface geometry should favour ruled surfaces and geodesic lines. Compound curvature should be intentional and structurally motivated, not arbitrary.

---

## Stage 2 -- Equipment Fabrication and Process Development
### Year 2

No body or chassis parts are made in Stage 2. Stage 2 builds and validates the equipment that makes Stage 3 possible. Getting the process wrong at full scale is catastrophic. Getting it wrong on a test panel costs an afternoon.

### 2.1 Large Format 3D Printer

**Purpose:** Print monocoque tooling forms -- sacrificial outer shells and permanent inner lattice cores -- body panel forms, and any large geometry components.

**Specification:**

- Minimum build volume: 1200 x 1200 x 600mm
- Print head: bought -- E3D or Slice Engineering high flow hotend, £400 budget
- Motion system: CoreXY architecture, linear rails, stepper motors
- Frame: aluminium extrusion, steel gussets at high load joints
- Heated bed: silicone heating pads tiled, zoned PID control for even temperature
- Controller: Klipper firmware on Raspberry Pi with standard motion control board
- Enclosure: insulated, actively heated to 50-70 degrees chamber temperature
- Estimated build cost: £2,500-3,000

**Primary materials:**

- ASA: UV resistant, good temperature tolerance, suitable for most tooling applications
- High temperature PEI or carbon filled nylon: for permanent lattice cores
- PC: for tooling requiring maximum rigidity

### 2.2 Forged Carbon Injection Machine

**Purpose:** Inject chopped carbon fibre and epoxy resin compound into closed printed moulds under controlled pressure and temperature.

**Architecture:** Screw drive, heated barrel, PID temperature control, tapered nozzle.

**Barrel specification:**

- Internal diameter: 50-80mm, length: 400-500mm
- Steel, bored and polished internally, hard chrome plated bore and screw
- Fabricated on existing CNC lathe
- Band heaters externally, PID controlled to 40-60 degrees
- Thermocouple embedded in barrel wall

**Screw specification:**

- Constant pitch Archimedes screw, close clearance to barrel wall
- Turned to close tolerance on existing lathe
- Manual operation via geared handle for development phase
- Stepper motor drive added for production phase with programmable injection speed profiles
- Fast sprue fill, slow controlled cavity fill, hold pressure phase

**Nozzle and mould connection:**

- Tapered nozzle self-sealing into matching tapered mould port under injection pressure
- Ball valve shutoff between barrel and nozzle
- Injection pressure and screw position logged per shot

### 2.3 Autoclave

**Purpose:** Cure cosmetic carbon components where surface quality demands it.

**Specification:**

- Internal diameter: minimum 1.2-1.5m, length: 2.5-3m
- Working pressure: 5-7 bar, design pressure 2x working pressure
- Vessel material: certified pressure vessel steel plate, rolled and welded
- Door: interrupted thread bayonet closure
- Heating: internal electric elements or circulated hot air
- Vacuum connection through vessel wall
- Pressure relief valves, independent high pressure cutoffs

**Safety requirement:** A pressure vessel engineer must review and sign off the design before first pressurisation. This is non-negotiable.

### 2.4 Mould System Development

**Outer mould (sacrificial):**

- Printed in ASA or PC
- Offset of inner form by target carbon wall thickness (3-4mm)
- Sealed with tooling gelcoat or high temperature coat before use
- PVA or wax release agent
- Injection port at lowest point, tapered to accept nozzle
- Distributed vent holes across entire surface

**Inner form (permanent lattice core):**

- Printed in carbon filled nylon or high temperature PEI
- Geometry derived from topology optimisation output
- Lattice cell geometry designed for injection compound flow continuity
- Bonds permanently into finished part
- Visible through translucent forged carbon skin

**Distributed vent hole system:**

Thousands of small holes distributed across the outer mould face. Air evacuates continuously ahead of the advancing compound front. Hole geometry tapered -- wider on cavity face, narrowing outward -- so compound plugs self-lock in the taper under injection pressure.

**Vent hole development test matrix:**

Print small mould sections with hole diameters of 0.3, 0.5, 0.8, and 1.0mm. Inject identical compound through each. Record which diameters allow compound penetration and which maintain air-only permeability. Defines hole diameter for all subsequent tooling.

**Compound starting specification:**

- Chopped carbon fibre: 25-50mm strand length
- Resin: West System or SP106 as starting point
- Mix ratio by weight: 40% resin, 60% fibre (~45% fibre volume fraction)
- Barrel temperature 40-60 degrees

### 2.5 Process Validation Test Panels

**Test sequence:**

- Panel 1: Flat 300x300mm, no lattice core. Establish compound, pressure, and cure baseline.
- Panel 2: Same geometry with simple honeycomb lattice core. Validate compound flow and vent system.
- Panel 3: Curved panel matching approximate monocoque curvature. Validate curved geometry behaviour.
- Panel 4: Full complexity at representative scale with chosen lattice geometry. Go/no-go decision for full scale tooling.

All test panels to materials testing lab. Results compared to Stage 1 rule of mixtures predictions. Correlation documented.

---

## Stage 3 -- Chassis and Drivetrain
### Year 3

A running, driving car by end of Stage 3. Mechanically complete. Structurally sound. Visually unfinished.

### 3.1 Monocoque Tub

**Construction:** Forged carbon injection over printed lattice core.

**Architecture:**

- Bathtub form: floor, sill boxes, front bulkhead, rear bulkhead, scuttle
- Rear arch geometry integrated into monocoque rear section
- Front and rear subframe hard points engineered into tub
- Drivetrain tunnel and cockpit as permanent obstacle geometry in optimisation
- Geometry output from Stage 1 topology optimisation software

**Torsional stiffness validation:**

Fix rear of tub to rigid fixture. Apply known weights at front corners via spreader bar. Measure deflection with dial gauges. Calculate actual torsional stiffness. Compare to simulation prediction. Document correlation.

### 3.2 Subframes

- Material: 4130 chromoly tube, TIG welded
- Front: carries suspension geometry, steering, engine mounting
- Rear: carries rear suspension, final drive
- Interface: bolted to monocoque hard points, designed for removal

### 3.3 Suspension

Double wishbone front and rear as target. Geometry defined in Stage 1. Pickup point locations fixed before monocoque tooling is produced -- these cannot change after the tub is made.

Bespoke uprights are a candidate for the forged carbon over printed core process, consistent with the build's manufacturing philosophy.

### 3.4 Engine and Drivetrain

**Primary candidate:** Yamaha R1 CP4 crossplane inline four. 200bhp, ~65kg, 14,000rpm, integral sequential gearbox.

**Secondary candidate:** Suzuki Hayabusa 1340cc. 197bhp stock, broader torque curve more suited to car application, ~75kg, extensive turbo aftermarket to 350-400bhp.

**Twin configuration:** Two R1 or Hayabusa units coupled to common output. 400bhp from 130-140kg. Established precedent. Significant engineering narrative value.

**KTM RC8 V-twin:** 170bhp, ~60kg, character and sound suited to hot rod aesthetic.

**Final decision criteria:** Power target 300-400bhp. Weight priority. Engine character. Engineering narrative value.

**Gearbox:** R1 integral sequential unit preferred -- proper sequential shift without standalone box cost. Short ratios require final drive adjustment via chain or belt primary reduction.

---

## Stage 4 -- Body and Finish
### Year 4

The car becomes visually what it was designed to be.

### 4.1 Body Panel System

**Non-structural outer skin panels (bonnet, wings, rear body, sills):**

Large format polymer print in ASA. Full panel size in one piece where machine volume permits. Sectioned at designed split lines where not. Filler and paint directly on printed surface after sealing. Cheap to reprint if damaged.

**Door structure (if applicable):**

LPBF aluminium from Shenzhen bureau. AlSi10Mg. Sectioned, shipped, TIG welded or adhesive bonded at joins. Estimated cost: $1,000-1,500 landed per door. Provides metal structure for hinge and latch points.

Note: if the final design maintains the open roadster configuration with no conventional doors, this requirement is eliminated entirely.

**Forged carbon components:**

Structural and cosmetic components deploying the visible lattice-through-carbon aesthetic. Produced using Stage 2 mould system and injection machine. Cured in fabricated autoclave where surface quality demands it.

### 4.2 Surface Treatment

- Forged carbon: clear coat over raw surface. Internal lattice geometry target: visible as ghost pattern beneath the carbon.
- Vent hole plugs: sanded flush and clear coated as micro-texture, or embraced as deliberate surface feature -- decision informed by test panel assessment in Stage 2.
- Polymer panels: conventional filler, primer, and paint process.

### 4.3 Interior

Consistent manufacturing philosophy throughout. Carbon and printed components carry the visual language of the exterior into the cockpit. Interior is a judged AMBR category.

### 4.4 Paint

Direction established by render: matte black. Final confirmation in Stage 1 design resolution.

The Triple Gun Award of Excellence is judged on paint and body fit and finish specifically. Panel gaps and surface execution are evaluated by a professional committee. Secondary target award.

---

## Stage 5 -- Buffer and Show Preparation
### Year 5

This year will be needed. Every serious build runs over.

**Show preparation tasks:**

- Display design: 20x20 display required for AMBR entry. Concept developed in Stage 1 alongside the car.
- Judges information book: designed as an object, not an afterthought. Contents listed below.
- Application deadline: November 1st preceding the show year.
- Show timing: late January or early February, Pomona Fairplex.
- Debut restriction: no finished-state imagery until after GNRS unveiling. Build coverage permitted throughout.
- Driving requirement: car must be driven before the judging panel at the start of judging. Mechanical reliability on that day is the single highest priority.

**Judges information book contents:**

- Project philosophy and design intent
- Software development: generative design system, constraint derivation, convergence visualisations
- Load case derivation and vehicle dynamics calculations
- Material property calculations versus measured test results
- Equipment fabrication: printer, injection machine, autoclave
- Process development: test panel series and results
- Monocoque fabrication with torsional stiffness validation
- Body system fabrication
- Drivetrain installation

**Display timelapse videos:**

- Topology optimisation converging in real time for the monocoque tub
- Topology optimisation for representative suspension components
- Injection process on a representative panel

**Potential academic publication:**

The combination of custom topology optimisation software with process-specific manufacturing constraints, and the novel forged carbon over printed core injection process, represents a genuine extension of the state of the art. A technical paper describing the methodology has publication potential. Hot Rod Magazine and an engineering journal covering the same project simultaneously has no precedent in AMBR history.

---

## Budget Philosophy

The low cost is part of the story. A full body in hand-formed aluminium from a skilled craftsman costs £80-150k in labour alone. This build replaces that with digital process and fabricated equipment.

**Rough cost envelope:**

| Item | Estimate |
|---|---|
| Print head and printer components | £3,000 |
| Injection machine materials | £2,000 |
| Autoclave materials | £10,000 |
| Donor engine | £5,000-8,000 |
| LPBF bureau (metal components) | £3,000-5,000 |
| Carbon fibre, resin, filament (full build) | £8,000-12,000 |
| Chromoly and fabrication materials | £3,000 |
| Materials testing | £1,000 |
| Miscellaneous and contingency | £5,000 |
| **Total estimate** | **£40,000-49,000** |

---

## Open Decisions

To be resolved in Stage 1:

- Final engine selection and twin versus single configuration
- Door configuration -- open roadster versus conventional doors
- Lattice geometry family (gyroid, Voronoi, geodesic, custom topologically optimised)
- Vent hole diameter (resolved by test panel matrix in Stage 2)
- Suspension geometry and pickup point locations
- Final drive configuration
- Interior specification
- Paint colour confirmation

---

*Project Blacktub -- Build Specification v1.0 -- March 2026*
