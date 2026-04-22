
# Implementation steps for the ship traffic generator with Bayesian Optimization (BO)

## 1. Fixed scenario decisions (set once per experiment, not optimized)
* Number of target ships
* Encounter type per target ship (head-on, crossing-give-way, crossing-stand-on, overtaking-give-way, overtaking-stand-on)
* Own ship parameters stay fixed from the CONFIG YAML (spawn, route, FMU params)

## 2. BO search space — trafficgen encounter params (per target ship)
These are passed into the `SituationInput` JSON and control the encounter geometry:
```python
# Per target ship encounter
{
    "desiredEncounterType": "<fixed from step 1>",
    "beta": <BO param, range or float>,          # relative bearing [deg]
    "relativeSpeed": <BO param, range or float>,  # ratio of TS speed to OS speed
    "vectorTime": <BO param, range or float>      # time to encounter [min]
}
```

## 3. BO search space — intermediate waypoints (per target ship)
After trafficgen produces the start/end positions, BO can inject intermediate waypoints:
* `wp_count`: number of intermediate waypoints (int, e.g. [0, 3])
* Per intermediate waypoint: north/east offset from trafficgen baseline [m]
* Per leg: speed [m/s]

## 4. Per BO trial: build `SituationInput` and call `generate_traffic_situations()`
* Construct `SituationInput` from fixed encounter types + BO-suggested `beta`, `relativeSpeed`, `vectorTime`
* Call `trafficgen.ship_traffic_generator.generate_traffic_situations()` with:
  - `situations_data`: the constructed `SituationInput`
  - `own_ship_data`: path to `data/ownship/ownship.json`
  - `target_ships_data`: path to `data/targetships/`
  - `settings_data`: path to encounter settings or default
* Returns `list[TrafficSituation]`

## 5. Extract and convert trafficgen output to co-sim units
For each ship (OS and each TS) in the `TrafficSituation`:
* **Positions**: waypoint lat/lon → NED meters using `trafficgen.marine_system_simulator.llh2flat(lat, lon, lat_0, lon_0)` where `lat_0, lon_0` = OS initial position (NED origin)
* **Heading**: `initial.heading` [deg] → `spawn.yaw_angle_deg`
* **Speed**: leg `sog` knots → m/s using `trafficgen.utils.knot_2_m_pr_s()`

## 6. Populate co-sim CONFIG with converted values
For each target ship, set:
* `spawn.north`, `spawn.east` ← trafficgen first waypoint NED position
* `spawn.yaw_angle_deg` ← heading from trafficgen output
* `spawn.forward_speed` ← first leg speed in m/s
* Route structure (ordered):
  1. **Start WP** ← trafficgen first waypoint (NED) — same as spawn position
  2. **Intermediate WPs** ← BO-suggested waypoints (NED offsets), if `wp_count > 0`
  3. **End WP** ← trafficgen last waypoint (NED) — the encounter goal position
* `route.north` = [start_north, ...intermediate_north..., end_north]
* `route.east` = [start_east, ...intermediate_east..., end_east]
* `route.speed` = leg speeds in m/s (one per leg, so `wp_count + 1` entries)

Own ship config remains unchanged from the baseline YAML.

## 7. Run simulation and compute metrics
* Build `ShipInTransitCoSimulation` from the populated config
* Call `instance.Simulate()`
* Extract safety metrics (dist, dcpa, tcpa, collision, fuel, etc.) using existing `compute_trial_metrics()`

## 8. Ax experiment loop
* Define Ax parameter space combining steps 2 and 3
* Sobol initialization trials → GP/BO trials
* Each trial: steps 4 → 5 → 6 → 7 → return objective to Ax
* Save best parameters and replay