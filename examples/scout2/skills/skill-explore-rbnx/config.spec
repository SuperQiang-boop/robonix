# Validated by explore_skill.configuration.ExploreSettings at Driver(CMD_INIT).
# Existing requests keep 0=unlimited and positive=explicit override semantics.
# Pilot passes -1 when the user has not specified a duration; this deployment
# defaults to no total deadline while per-leg and no-progress limits remain.
config:
  explore_mode: frontier
  timeout_s: 0                  # unlimited total task time, selected by timeout_s=-1
  nav_goal_timeout_s: 180       # hard per-leg action cap; estimates are advisory below this
  nav_goal_min_timeout_s: 30
  nav_time_factor: 1.5          # multiplies route length / live Nav2 speed limit
  nav_turn_allowance_s: 30      # turn/deceleration/settling allowance
  sweep_every_n_legs: 0         # 0 disables all autonomous in-place sweeps
  arrival_tolerance_m: 0.45     # exploration observation radius, not Nav2's goal tolerance
  arrival_settle_s: 2
  nav_stall_timeout_s: 20       # no observed translation; initial turn allowance is separate
