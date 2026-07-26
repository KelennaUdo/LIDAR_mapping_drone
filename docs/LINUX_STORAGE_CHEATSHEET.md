# Linux Storage and Mounting Cheat Sheet

## Mental Model

```text
Physical Seagate drive
└── NTFS partition
    └── PX4_LINUX_WORKSPACE.img (30 GB file)
        └── ext4 Linux filesystem
            └── /mnt/px4-workspace/PX4-Autopilot
```

A **mount point** is a directory where Linux makes a filesystem accessible.
Mounting does not copy the files. It connects a filesystem to a directory.

## Normal Project Workflow

Use the project helper instead of manually managing the two storage layers:

```bash
# Beginning of a PX4 session: connect and verify both drives.
./scripts/px4_workspace.sh connect

# Inspect storage and PX4 state without changing mounts.
./scripts/px4_workspace.sh status

# End of a PX4 session: stop PX4, unmount, and eject safely.
./scripts/px4_workspace.sh disconnect
```

The script must be run as your normal user. It requests `sudo` itself when
Linux needs administrator permission.

The remainder of this cheat sheet explains the commands used underneath the
helper and is useful for diagnosis.

## Project Paths

```text
External NTFS mount:
/run/media/kelenna-udo/Seagate Backup Plus drive

30 GB ext4 image:
/run/media/kelenna-udo/Seagate Backup Plus drive/PX4_LINUX_WORKSPACE.img

Mounted ext4 workspace:
/mnt/px4-workspace

PX4 source:
/mnt/px4-workspace/PX4-Autopilot

Docker internal storage:
/var/lib/docker

Project repository:
/home/kelenna-udo/LIDAR_mapping_drone
```

## Inspect Physical Storage

```bash
# List disks, partitions, filesystem types, labels, and mount points.
lsblk -o NAME,SIZE,FSTYPE,LABEL,MOUNTPOINTS

# Show mounted filesystems in a tree.
findmnt

# Show the filesystem containing one path.
findmnt -T /mnt/px4-workspace

# Show free space on every mounted filesystem.
df -h

# Show free space specifically for the internal root filesystem.
df -h /

# Show free space in the external ext4 workspace.
df -h /mnt/px4-workspace

# Measure the PX4 checkout's current space usage.
du -sh /mnt/px4-workspace/PX4-Autopilot

# List active loop devices and their backing image files.
losetup -l
```

`lsblk` device names such as `/dev/sdb1` can change after reconnecting hardware.
Use the filesystem label and mount path to identify the intended drive.

## Manual Troubleshooting: Mount the External NTFS Partition

First identify the device with `lsblk`. If it is `/dev/sdb1`:

```bash
# Ask the desktop storage service to mount the physical partition.
udisksctl mount -b /dev/sdb1

# Confirm where that partition was mounted.
findmnt -rn -S /dev/sdb1 -o SOURCE,TARGET,FSTYPE,OPTIONS
```

Expected filesystem type:

```text
ntfs3
```

The expected path contains spaces, so quote it whenever it is used in a shell
command.

## Manual Troubleshooting: Mount the 30 GB ext4 Image

```bash
# Store the long image path in a shell variable.
IMAGE="/run/media/kelenna-udo/Seagate Backup Plus drive/PX4_LINUX_WORKSPACE.img"

# Create the empty directory that will expose the mounted filesystem.
sudo mkdir -p /mnt/px4-workspace

# Connect the image to a loop device and mount its ext4 filesystem read-write.
sudo mount -o loop,rw "$IMAGE" /mnt/px4-workspace

# Give the current user ownership of the workspace root.
sudo chown "$USER:$USER" /mnt/px4-workspace
```

Verify without creating files:

```bash
# Show the source device, mount point, filesystem, and options.
findmnt -T /mnt/px4-workspace \
  -o SOURCE,TARGET,FSTYPE,OPTIONS

# Confirm that the current user can write to the mounted directory.
test -w /mnt/px4-workspace \
  && echo "Workspace is writable" \
  || echo "Workspace is not writable"

# Confirm the available capacity.
df -h /mnt/px4-workspace
```

Expected filesystem type:

```text
ext4
```

## After Restarting the Computer

Files remain saved, but mounts are temporary connections and usually need to
be recreated:

```text
Physical files on Seagate drive: remain
PX4 source and build output: remain
Docker images on internal drive: remain
NTFS mount: may need to be recreated
ext4 image mount: needs to be recreated
running containers: stopped during shutdown
```

Mount the physical partition first, then mount the ext4 image.

## Manual Troubleshooting: Safe Unmount Order

Stop PX4 before unmounting:

```bash
# Check for a running PX4 container.
sudo docker ps --filter name=px4-sitl

# If present, request a graceful stop.
sudo docker stop --timeout 30 px4-sitl

# Disconnect the ext4 workspace from /mnt/px4-workspace.
sudo umount /mnt/px4-workspace

# After confirming the correct device name, unmount the physical partition.
udisksctl unmount -b /dev/sdb1
```

Do not unplug the drive until both mounts are gone.

## Diagnosing "Target Is Busy"

The filesystem cannot be unmounted while a process is using it:

```bash
# Check whether a terminal or process has files open in the workspace.
fuser -vm /mnt/px4-workspace

# Confirm that no PX4 container remains.
sudo docker ps --filter name=px4-sitl

# Move the current terminal out of the mounted directory.
cd "$HOME"
```

Stop the identified process normally. Avoid force-unmounting a writable
workspace because buffered data may not be written safely.

## Storage Versus Memory

```text
Disk/storage: persistent files measured by df, du, and docker system df
RAM/memory: temporary working space used by running programs
GPU memory: temporary graphics/compute memory shown by nvidia-smi
```

Stopping a container releases RAM and GPU memory. It does not delete the
Docker image, PX4 source, or PX4 build output.
