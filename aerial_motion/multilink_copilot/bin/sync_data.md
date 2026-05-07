# bin scripts usage

This directory contains three experiment helper scripts for recording rosbags,
saving experiment setup snapshots, and manually synchronizing experiment data.

Unless noted otherwise, start from the script directory:

```bash
roscd multilink_copilot/bin
```

## Overview

| Script | Purpose |
| --- | --- |
| `rosbag.sh` | Records the standard multilink copilot experiment topic set to `multilink_copilot/data/rosbag`. |
| `save_exp_setup.sh` | Copies the current launch and YAML configuration files to `multilink_copilot/data/exp_setup`. |
| `sync_data.sh` | Manually synchronizes local `multilink_copilot/data` with a private remote data directory through `rclone bisync`. |

## `rosbag.sh`

Records the standard topic set used by multilink copilot experiments.

The default robot namespace is `dragon`:

```bash
bash rosbag.sh
```

Use a different robot namespace:

```bash
bash rosbag.sh <robot_ns>
```

Pass extra options directly to `rosbag record` after the namespace. When passing
extra options, always provide the namespace explicitly:

```bash
bash rosbag.sh dragon -O four_ring_trial.bag
```

Recorded bags are saved under:

```bash
multilink_copilot/data/rosbag
```

The script creates the output directory if it does not already exist. Stop
recording with `Ctrl-C`.

## `save_exp_setup.sh`

Saves a snapshot of the current experiment setup files:

```bash
bash save_exp_setup.sh
```

The script creates a new output directory under:

```bash
multilink_copilot/data/exp_setup
```

The output directory name uses the current date plus an incrementing suffix:

```text
YYYYMMDD_001
YYYYMMDD_002
...
```

Each run copies the following files:

- `multilink_copilot/launch/copilot_planner.launch`
- `multilink_copilot/config/copilot_planner.yaml`
- `mono_planner/launch/waypoint_conditioned_planner.launch`

Run this before or after a trial when the launch and configuration files should
be preserved together with the recorded data.

## `sync_data.sh`

Manually synchronizes:

- local `multilink_copilot/data`
- a private remote data directory configured outside git

### Initial Setup

Copy the local-only config template:

```bash
cp sync_data.local.env.example sync_data.local.env
```

Edit `sync_data.local.env` and set:

- `MULTILINK_COPILOT_SYNC_REMOTE_DATA_DIR`
- `MULTILINK_COPILOT_SYNC_REMOTE_BACKUP_DIR`

Example:

```bash
MULTILINK_COPILOT_SYNC_REMOTE_DATA_DIR="gdrive:your/private/data/path"
MULTILINK_COPILOT_SYNC_REMOTE_BACKUP_DIR="gdrive:your/private/backup/path"
```

Requirements:

- `rclone >= 1.71.0`
- `rclone` has `bisync` support
- the default `rclone` path is `~/.local/bin/rclone`
- the remote, for example `gdrive:`, is already configured in `rclone`

Use a different `rclone` binary if needed:

```bash
RCLONE_BIN=/path/to/rclone bash sync_data.sh check
```

Use a different config file if needed:

```bash
MULTILINK_COPILOT_SYNC_CONFIG_FILE=/path/to/sync.env bash sync_data.sh sync
```

### Commands

Initialize the remote directories and `bisync` state for the first time:

```bash
bash sync_data.sh init
```

Run a normal bidirectional sync:

```bash
bash sync_data.sh sync
```

Rebuild the `bisync` baseline:

```bash
bash sync_data.sh resync
```

Check whether local and remote data match:

```bash
bash sync_data.sh check
```

### Sync Behavior

- The local sync directory is `multilink_copilot/data`.
- Sync filters are loaded from `sync_data.filters`.
- The script refuses to sync if `*.active` files exist under the local data directory.
- Conflicts are resolved by newer timestamp; the older side is preserved with the `conflict` suffix.
- Local backups are stored under `~/.local/share/multilink_copilot_sync_backup/data_local`.
- Remote backups are stored under `MULTILINK_COPILOT_SYNC_REMOTE_BACKUP_DIR`.

If `sync_data.filters` changes, run this before the next normal `sync`:

```bash
bash sync_data.sh resync
```

### Local-only Files

The following private or generated files are ignored by git:

- `sync_data.local.env`
- `sync_data.filters.md5`

Do not run `sync_data.sh` while `rosbag record` or CSV post-processing is
actively writing into `multilink_copilot/data`.
