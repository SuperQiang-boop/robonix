# Runtime configuration for the Unitree G1 speaker primitive.

config:
  # Network interface connected to the G1 control network. Must match the
  # interface used by the other Unitree SDK2 providers.
  network_interface: enp7s0

  # Maximum PCM bytes buffered for one speaker request. Requests above this
  # limit fail rather than growing process memory without bound.
  max_audio_bytes: 33554432

  # Maximum time to wait for SDK2 initialization and the initial volume probe.
  startup_timeout_s: 15.0
