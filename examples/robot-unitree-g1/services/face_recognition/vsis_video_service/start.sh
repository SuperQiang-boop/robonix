#!/bin/bash
set -e

mkdir -p /vsis_video_service/person/photo
mkdir -p /vsis_video_service/person/db
mkdir -p /vsis_video_service/logs

exec /vsis_video_service/bin/vsis_video_service