# Runtime configuration for the Unitree G1 audio primitive.

config:
  # Network interface connected to the G1 control network. Must match the
  # interface used by the other Unitree SDK2 providers.
  network_interface: enp7s0

  # Maximum PCM bytes buffered for one speaker request. Requests above this
  # limit fail rather than growing process memory without bound.
  max_audio_bytes: 33554432

  # Maximum time to wait for SDK2 initialization and the initial volume probe.
  startup_timeout_s: 15.0

  # G1 voice-service multicast channel used by the onboard microphone.
  mic_multicast_group: 239.168.123.161
  mic_multicast_port: 5555

  # Local IPv4 address used only for multicast membership. UDP reception still
  # binds 0.0.0.0:<mic_multicast_port> so multicast datagrams are delivered.
  mic_bind_address: ""

  # PCM metadata carried in AudioChunk.duration_s. The G1 stream is normally
  # 16 kHz mono signed 16-bit little-endian PCM.
  mic_sample_rate: 16000
  mic_channels: 1
  mic_bits_per_sample: 16
  mic_max_packet_bytes: 65535
  mic_socket_timeout_s: 0.5
