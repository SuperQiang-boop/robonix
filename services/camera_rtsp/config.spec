specVersion: 1
description: >-
  Camera source and RTSP publication settings for the camera RTSP service.
  Applied once at Driver INIT; a repeated INIT with a different resolved
  configuration fails. Dimensions and encoding come from the first received
  Image, so a change of either needs a reactivation.

properties:
  camera_provider_id:
    type: string
    description: >-
      Provider id of the ROS 2 RGB camera whose stream is republished. The
      service resolves it through Atlas and refuses to start when no single
      provider answers, so a renamed or duplicated camera fails loudly instead
      of silently publishing the wrong image.

  rtsp_url:
    type: string
    description: >-
      Destination rtsp://host:port/path. Credentials, query and fragment are
      rejected. With manage_server the host must be loopback, because the
      MediaMTX instance the service starts is not exposed to the network.

  manage_server:
    type: boolean
    description: >-
      Run an owned MediaMTX server on loopback. Set false to publish into an
      RTSP server that already exists, including a remote one.
    default: true

  fps:
    type: integer
    description: >-
      Maximum output frame rate. Higher-rate input is decimated; the encoder
      never interpolates.
    default: 15
    minimum: 1
    maximum: 60

  startup_timeout:
    type: number
    description: >-
      Seconds allowed for the encoder plus the decoded-frame probe before
      activation fails. Bounded so a missing ffmpeg cannot hang boot. The
      service itself rejects 0.
    default: 25
    minimum: 0
    maximum: 120

  frame_timeout:
    type: number
    description: >-
      Seconds without a published frame, or blocked on a stalled RTSP writer,
      before the stream is reported unavailable. The service itself rejects 0.
    default: 5
    minimum: 0
    maximum: 120
