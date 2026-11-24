# Method comparison: DRAFT

## 1. Purpose

This fixes one set of search parameters that every test method in the paper reads, so that the groups are compared on the same scenario, the same own ship and the same score.

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
  scenario.yaml          search space, traffic generator settings, run length, seed;
                         points to the simulation config
  disturbance.yaml       route-disturbance limits
  failure_criteria.yaml  failure events and the shared run score
```

Each value is a plain YAML value with its unit in a comment. Each section names the file it was taken from. Most values come from the current AST setup on main.


## 3. Open decisions

1. **Seed scheme:** one scheme for case sampling, trafficgen and the
   wind/current FMUs, shared by all methods.
2. **Run score definition:** today the score is the mean over target ships
   of −ln(1 + d − 100), with d the final distance. Two problems:
   - **Final distance:** most runs end when the own ship reaches its end
     waypoint, often long after the encounter. A close pass that stays above
     100 m then scores as safe.
   - **Mean over ships:** a far-away ship hides a near miss on the other. 

   Proposal: score = −ln(1 + d_min − 100), with d_min the closest the own ship
   gets to any target ship over the whole run. A collision then gives 0
   automatically; a navigation failure still gives 0 explicitly.
3. **Ties in the score:** when an own-ship failure and a target-ship end event
   happen at the same step, main gives the distance score instead of 0.
4. **Group 4 sampling:** how angle and length are drawn within the limits
   (e.g. uniform), and when changes are triggered.
5. **Encounter type in BO:** handle it as a categorical dimension, or run one
   GP per type pair?
6. **Wind and current:** their mean inputs are never set only the initial values, so means decay to 0. Seed 0 also gives the same noise in every run.
   How do we want to handle this?
7. **Approach 6:** include it or not.
8. **Runs per approach:** the budget N for each approach. How do we count this?
9. **Issues on main** (for awareness):
   - the interception reward wraps a degree difference with a radian wrap
     (`main:EBASTv2_core/reward_function.py:332`);
   - current parameters match spawn_request_bank_ebastv2.pkl, but any changes to the parameters would need a new bank
   - current bank does not have a seed so rerunning the code will create a new bank
