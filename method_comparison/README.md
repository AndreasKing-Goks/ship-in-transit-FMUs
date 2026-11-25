# Method comparison: DRAFT

## 1. Purpose

This fixes the simulation setup and search parameters that every test method in the paper reads, so that the groups are compared on the same scenario, the same own ship and the same score.

### Simulation groups

| #   | Approach                                            | Starting encounters                                                                                                                         | Route disturbances                            |
|-----|-----------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------|
| 1   | Sobol + Bayesian optimization (BO)                  | Sobol, then BO; builds a Gaussian process (GP) surrogate and splits the encounter space into predicted safe, critical and uncertain regions | none                                          |
| 2   | GP-selected starts + Adaptive Stress Testing (AST)  | from the GP's predicted safe and uncertain regions                                                                                          | AST                                           |
| 3   | Random starts + AST                                 | random                                                                                                                                      | AST                                           |
| 4   | GP-selected starts + random admissible disturbances | from the GP's predicted safe and uncertain regions                                                                                          | random, within the AST limits                 |
| 5   | Monte Carlo (MC)                                    | random                                                                                                                                      | none                                          |
| (6) | Random starts + random admissible disturbances      | random                                                                                                                                      | random, within the AST limits (**undecided**) |

Key comparisons: 2 vs 3, 2 vs 4, and 1+2 vs 5.

"No disturbances" means no intermediate-waypoint disturbances. Wind and current are the same in all groups.

## 2. File layout

```
method_comparison/
  README.md              this file
  config/                configuration files
```

## 3. Open decisions

1. **Seed scheme:** we need to decide on a scheme for case sampling, trafficgen and the
   wind/current FMUs to ensure reproducibility.
   - All wind and current FMUs use the same seed at the moment.
   - No seed for bank generation. Does not matter in practice if the same bank is re-used.
   - Not sure if the AST train/test split and training use a seed.
2. **Run score definition:** currently the score is the mean over target ships
   of −ln(1 + d − 100), with d the final distance. Two problems:
   - **Final distance:** most runs end when the own ship reaches its end
     waypoint, often long after the encounter. A close pass that stays above
     100 m then scores as very safe.
   - **Mean over ships:** a far-away ship hides a near miss on the other.
   
   Proposal: score = 0 if d_min < 100, else −ln(1 + d_min − 100), with d_min the closest the own ship
   gets to any target ship over the whole run.
3. **Ties in the score:** when an own-ship failure and a target-ship end event
   happen at the same step, main gives the distance score instead of 0. I would expect 
   a value to be 0 in this case.
4. **Group 4 sampling:** how angle and length are drawn within the limits
   (e.g. uniform), and when changes are triggered.
5. **Encounter type in BO:** handle it as a categorical dimension, or run one
   GP per type pair?
6. **Wind and current:** their mean inputs are never set, only the initial values, so the means decay to 0. Seed 0 also gives the same noise in every run.
   How do we want to handle this?
7. **Approach 6:** include it or not.
8. **Runs per approach:** the budget N for each approach. How do we count this?
9. **Issues on main** (for awareness):
   - the interception reward wraps a degree difference with a radian wrap
     (`main:EBASTv2_core/reward_function.py:332`);
   - the current parameters match spawn_request_bank_ebastv2.pkl, but any change to the parameters would need a new bank;
   - the current bank does not have a seed, so rerunning the code will create a new bank.
10. **Trafficgen:** it randomly samples a meeting point within a radius, even with all other parameters fixed.
    - That knob is hidden from BO, so it essentially behaves as noise.
    - It does not really matter for AST or MC.
    - We could set the radius to 0 to make trafficgen deterministic. The downside is that all samples will then be on a collision course. The upside is that the COLAV system would have to act in all simulations.
11. **Retraining:** is the plan to retrain the AST model?

| #   | Decision |
| --- | -------- |
| 1   |          |
| 2   |          |
| 3   |          |
| 4   |          |
| 5   |          |
| 6   |          |
| 7   |          |
| 8   |          |
| 9   |          |
| 10  |          |
| 11  |          |

## 4. Protocol

Basic rules to follow when running the analyses for the paper.

1. **Question:** what do we want to find out?
   ___
2. **Methods:** 
   See simulation groups above.

   The basic principle is to compare how these methods perform in identifying critical scenarios.

3. **Same for all:** what must be the same in every group?
   - simulation setup: `method_comparison/config/simulation.yaml`
   - traffic generator settings: `method_comparison/config/encounter_settings.json`
   - hardcoded simulation parameters: `method_comparison/config/hardcoded_parameters.yaml`
   - Perhaps a pinned version of the code
4. **What each method may change:** methods differ only in how they pick the
   inputs below.

   **A. Starting encounter** (set at the start of each run)
   - per target ship: encounter type, relative bearing (β), relative speed,
     vector time
   - own ship: speed and heading
   - all values must stay inside the ranges in `config/`

   **B. Route disturbances** (picked during a run)
   - per target ship: up to 5 extra waypoints, each a scope angle and a
     scope length, inside the limits in `config/simulation.yaml`

   | Group | Picks A by               | Picks B by       |
   |-------|--------------------------|------------------|
   | 1     | Sobol, then BO           | none             |
   | 2     | GP regions from group 1  | AST              |
   | 3     | random                   | AST              |
   | 4     | GP regions from group 1  | random           |
   | 5     | random                   | none             |
   | (6)   | random                   | random           |

   - "random" always means the same shared sampler, so groups 3, 5 and 6 draw
     starts from the same distribution.
   - Groups 2 and 4 use the same GP-selected starts, so 2 vs 4 only compares
     AST with random disturbances.

5. **Score:** how do we score one run?
   ___
6. **Failure:** what counts as a failure?
   ___
7. **Budget:** how many simulations may each method use? Does AST training count?
   ___
8. **Repeats and seeds:** how many times do we run each method, and with which seeds?
   ___
9. **What we save from each run:** inputs, seeds, score, failure flags, ___
10. **How we compare:** which numbers and plots do we show?
    ___
