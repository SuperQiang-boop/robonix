# Driver INIT object; unknown keys rejected. Paths resolve against RBNX_INVOCATION_CWD.
# Required: mapping_provider_id, lidar_provider_id, odom_provider_id.
# map_id is optional; when present it restricts the loaded mapping snapshot.
# Required: maps_dir (saved snapshots), index_dir (immutable prepared artifacts).
# index_dir must be outside maps_dir.
# Production ACTIVATE also requires profile_file and coordination_dir, supplied together.
# profile_file JSON: capture{scan_duration_ns,header_is_end,...},
# base_from_sensor[16], registration{base_z,roll,pitch,...}; optional search,matching.
# The service resolves an exact map_id/generation/database_sha256 index at runtime.
# All timing/calibration values must be verified for the real driver and saved map.
# coordination_dir: same-host shared directory mounted into mapping/navigation.
# Relative paths resolve against RBNX_INVOCATION_CWD, as do all other paths.
# base_frame=base_link; odom_frame=odom; these must differ. use_sim_time=false.
# auto_relocalize: boolean, default true; configured in the service manifest.
# True enables automatic localization requests
# after mapping publishes a ready localization snapshot. False leaves the
# service active for status/map monitoring and waits for relocalize_by_map RPC.
# timeout_s in (0,120], default 15; request 0 selects this default.
# status_period_s in (0,5], default 1; Scout2 uses 0.1.
# The service may start during mapping; it waits without requesting localization
# until mapping publishes a ready localization snapshot. ACTIVE means resources
# ready. Only completed handoff plus fresh live verification grants
# localization_valid. Automatic retries are bounded to three per ready load.
