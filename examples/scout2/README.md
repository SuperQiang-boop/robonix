# Scout2

Scout2 deploys native ROS 2 primitives, services, and skills through
`robonix_manifest.yaml`. Run `rbnx setup` from the Robonix repository root,
then run `rbnx boot` from this directory after preparing the packages and hardware.

The `scout2_camera`, `scout2_lidar`, `scout2_chassis`, `move`, `patrol`, and
`say_hello` scripts use an activated Python virtual environment when `VIRTUAL_ENV` is set,
otherwise they use the `robonix-scout2` Conda environment. Prepare either
environment with Python 3.10 and the package dependencies for the current user.
The scripts find Conda through `CONDA_EXE`, then PATH, then installations under
`$HOME` or `/opt`. Set `CONDA_EXE` to the Conda executable for a custom installation.
A missing environment fails with Conda's diagnostic; it is not created at boot.

The three Scout2 primitive bootstraps use their selected interpreter's Python
packages and add ROS paths under `/opt/ros/${ROS_DISTRO:-humble}`. They resolve the
Robonix API source with `rbnx path robonix-api`; set `ROBONIX_API` to the directory
containing `robonix_api` to override that path. No previous user's Python or
Robonix checkout is required. The robot-description and AgileX chassis packages
use `python3` on PATH; activate the appropriate environment before booting.

For a native ROS 2 Humble host without Conda, prepare and activate an environment
from this directory (the environment shares system ROS packages):

```bash
uv venv --python /usr/bin/python3 --system-site-packages .venv
uv pip install --python .venv/bin/python -e ../../pylib/robonix-api 'numpy<2' Pillow
source .venv/bin/activate
bash scripts/check_python_env.sh
rbnx boot
```

Before boot, confirm ROS Domain 42 publishes actual RGB frames on
`/camera/camera/color/image_raw` and LaserScan frames on `/scan`; a topic
name alone is insufficient. Both primitive initializations require a frame.
