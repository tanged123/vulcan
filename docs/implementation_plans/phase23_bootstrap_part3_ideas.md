# Bootstrap Part 3: Feature Ideas from Aerosandbox

Research document for potential Vulcan additions based on validated physics models from [AeroSandbox](https://github.com/peterdsharpe/AeroSandbox).

---

## High Priority Candidates

### 1. Viscous Aerodynamics Library
**Source**: `aerosandbox/library/aerodynamics/viscous.py`

| Model | Description | Reference |
|-------|-------------|-----------|
| `Cd_cylinder(Re, M)` | Cylinder drag with Mach effects | Experimental fits |
| `Cf_flat_plate(Re, method)` | Skin friction (Blasius, turbulent, hybrid) | Cengel & Cimbala, Schlichting |
| `Cd_flat_plate_normal()` | Flat plate perpendicular to flow | Tian et al. (2014) |
| `fuselage_upsweep_drag()` | Aft-fuselage separation drag | Raymer §12.36 |

**Value**: Enables proper drag buildup. Currently Vulcan has only simple aero coefficients.

---

### 2. Transonic/Compressibility Aerodynamics
**Source**: `aerosandbox/library/aerodynamics/transonic.py`

| Model | Description | Reference |
|-------|-------------|-----------|
| `sears_haack_drag(r_max, L)` | Ideal wave drag for supersonic bodies | Sears-Haack theory |
| `mach_crit_Korn(CL, t/c, sweep)` | Critical Mach prediction | Mason §7.5.2 |
| `Cd_wave_Korn(CL, t/c, M)` | Wave drag coefficient | Mason §7.5.2 |
| `approximate_CD_wave(M, M_crit)` | Transonic wave drag model | Raymer §12.5.10 |

**Value**: Essential for high-speed vehicle design (Mach > 0.7). Well-validated empirical relations.

---

### 3. Normal Shock Relations
**Source**: `aerosandbox/library/aerodynamics/normal_shock_relations.py`

| Model | Description |
|-------|-------------|
| `mach_number_after_normal_shock(M, γ)` | Post-shock Mach |
| `density_ratio_across_normal_shock(M, γ)` | ρ₂/ρ₁ |
| `temperature_ratio_across_normal_shock(M, γ)` | T₂/T₁ |
| `pressure_ratio_across_normal_shock(M, γ)` | P₂/P₁ |
| `total_pressure_ratio_across_normal_shock(M, γ)` | Pt₂/Pt₁ |

**Value**: Standard gasdynamics for supersonic/hypersonic analysis. Simple, high utility.

---

### 4. Unsteady Aerodynamics
**Source**: `aerosandbox/library/aerodynamics/unsteady.py`

| Model | Description | Reference |
|-------|-------------|-----------|
| `wagners_function(s)` | Circulatory lift buildup | Jones (1939) |
| `kussners_function(s)` | Gust response function | Sears & Sparks (1941) |
| `indicial_pitch_response(s, α)` | Lift from impulsive pitch |  |
| `indicial_gust_response(s, w_g, V)` | Lift from sharp-edge gust |  |
| `added_mass_due_to_pitching(s, α(t))` | Added mass lift term |  |

**Value**: Gust loads, flutter preliminaries, dynamic maneuver analysis. Complements `transfer_functions`.

---

### 5. Solar Power & Airmass
**Source**: `aerosandbox/library/power_solar.py`

| Model | Description | Reference |
|-------|-------------|-----------|
| `solar_flux_outside_atmosphere(day)` | AM0 flux with orbital eccentricity | |
| `declination_angle(day)` | Solar declination from day of year | PV Education |
| `solar_elevation_angle(lat, day, time)` | Sun position for observer | PV Education |
| `solar_azimuth_angle(lat, day, time)` | Solar compass direction | PV Education |
| `airmass(elevation, altitude)` | Relative airmass | Young (1994) |
| `solar_flux(...)` | Flux on tilted panel with atm absorption | Earthscan (2008) |

**Value**: Solar-powered aircraft, array sizing. Could extend `environment` module.

---

### 6. Structural Buckling
**Source**: `aerosandbox/structures/buckling.py`

| Model | Description | Reference |
|-------|-------------|-----------|
| `column_buckling_critical_load(E, I, L, BC)` | Euler column buckling | Steel Construction Manual |
| `thin_walled_tube_crippling(E, t, r)` | Tube crippling load | Raymer §14.33 |
| `plate_buckling_critical_load(L, w, t, E)` | Plate buckling | NACA TN3781 |

**Value**: Structural sizing for tanks, fuselage panels, truss members.

---

### 7. Induced Drag & Wing Efficiency
**Source**: `aerosandbox/library/aerodynamics/inviscid.py`

| Model | Description | Reference |
|-------|-------------|-----------|
| `induced_drag(L, b, q, e)` | Induced drag force | Lifting line theory |
| `oswalds_efficiency(λ, AR, Λ)` | Oswald factor estimation | Nita & Scholz (2012) |
| `CL_over_Cl(AR, M, Λ)` | 3D lift from 2D data | Raymer §12.4.1 |
| `ground_effect_ratio(h/b)` | Ground effect drag reduction | Phillips & Hunsaker |

**Value**: Wing preliminary design and aircraft configuration trades.

---

### 8. Field Length Analysis
**Source**: `aerosandbox/library/field_lengths.py`

| Model | Description | Reference |
|-------|-------------|-----------|
| `field_length_analysis_torenbeek(...)` | Complete takeoff/landing | Torenbeek §5.4.5 |

**Output**: Ground roll, airborne distance, balanced field length, V_stall, V_liftoff, climb gradients.

**Value**: Aircraft mission analysis, runway requirements, engine-out scenarios.

---

## Medium Priority

### 9. Weight Estimation
**Source**: `aerosandbox/library/weights/`

Files: `raymer_cargo_transport_weights.py`, `torenbeek_weights.py`, etc.

**Value**: Quick-turnaround weight estimation for conceptual design.

### 10. Turbofan/Propeller Models
**Source**: `aerosandbox/library/propulsion_*.py`

| Model | Description |
|-------|-------------|
| `thrust_turbofan(mass)` | Max thrust from engine mass |
| `TSFC_turbofan(mass, BPR)` | Fuel consumption estimate |
| `mass_turbofan(m_dot, OPR, BPR, D)` | Engine weight (TASOPT/Drela) |
| `propeller_shaft_power_from_thrust(...)` | Actuator disk power |
| `mass_gearbox(P, rpm_in, rpm_out)` | Gearbox weight |

**Value**: Conceptual propulsion sizing. Extends current `vulcan/propulsion`.

### 11. Component Drag Buildup
**Source**: `aerosandbox/library/aerodynamics/components.py`

| Model | Reference |
|-------|-----------|
| `CDA_control_linkage(Re, L)` | Hepperle/Würz (1989) |
| `CDA_control_surface_gaps(...)` | Hoerner (1965) |
| `CDA_protruding_bolt_or_rivet(D)` | Hoerner (1965) |

**Value**: Detailed drag accounting based on Hoerner experimental data.

---

## Recommended Implementation Groupings

| Group | Contents | Effort | New Headers |
|-------|----------|--------|-------------|
| **A: Gasdynamics** | Normal shock, isentropic flow, Sears-Haack | 1 day | 1-2 |
| **B: Drag Library** | Flat plate Cf, Cylinder Cd, Korn wave drag | 2 days | 2-3 |
| **C: Unsteady Aero** | Wagner, Kussner, Duhamel utilities | 2-3 days | 2 |
| **D: Solar Power** | Solar position, airmass, panel flux | 2 days | 2-3 |
| **E: Buckling** | Column, tube, plate buckling | 1-2 days | 1-2 |
| **F: Aircraft Perf** | Oswald, induced drag, field lengths, weights | 3-4 days | 3-4 |

---

## Janus Compatibility Notes

All models should:
1. Template on `Scalar` for AD support
2. Use `janus::` math functions
3. Use `janus::where()` for scalar-dependent branching

Aerosandbox already uses symbolic-friendly patterns (softmax blending, smooth approximations).

---

## Questions for User

1. Which group(s) should we prioritize for Bootstrap Part 3?
2. Should we create a new `structures` module or add buckling to `mass`?
3. Preference on solar power: new module or extend `environment`?
4. Interest in aircraft performance (field lengths, weights) or more rocket-focused?

---

*Created: 2025-12-23*
*Reference: `/home/tanged/sources/vulcan/reference/AeroSandbox/`*
