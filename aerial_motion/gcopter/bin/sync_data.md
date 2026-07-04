# bin scripts usage

This directory contains the helper script for manually synchronizing gcopter
experiment data with a private remote data directory.

Unless noted otherwise, start from the script directory:

```bash
roscd gcopter/bin
```

## Overview

| Script | Purpose |
| --- | --- |
| `sync_data.sh` | Manually synchronizes local `gcopter/data` with a private remote data directory through `rclone bisync`. |

## `sync_data.sh`

Manually synchronizes:

- local `gcopter/data`
- a private remote data directory configured outside git

### Initial Setup

Copy the local-only config template:

```bash
cp sync_data.local.env.example sync_data.local.env
```

Edit `sync_data.local.env` and set:

- `GCOPTER_SYNC_REMOTE_DATA_DIR`
- `GCOPTER_SYNC_REMOTE_BACKUP_DIR`

Example:

```bash
GCOPTER_SYNC_REMOTE_DATA_DIR="gdrive:your/private/data/path"
GCOPTER_SYNC_REMOTE_BACKUP_DIR="gdrive:your/private/backup/path"
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
GCOPTER_SYNC_CONFIG_FILE=/path/to/sync.env bash sync_data.sh sync
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

- The local sync directory is `gcopter/data`.
- Sync filters are loaded from `sync_data.filters`.
- The script refuses to sync if `*.active` files exist under the local data directory.
- Conflicts are resolved by newer timestamp; the older side is preserved with the `conflict` suffix.
- Local backups are stored under `~/.local/share/gcopter_sync_backup/data_local`.
- Remote backups are stored under `GCOPTER_SYNC_REMOTE_BACKUP_DIR`.

If `sync_data.filters` changes, run this before the next normal `sync`:

```bash
bash sync_data.sh resync
```

### Local-only Files

The following private or generated files are ignored by git:

- `sync_data.local.env`
- `sync_data.filters.md5`

Do not run `sync_data.sh` while a producer is actively writing into
`gcopter/data`.
