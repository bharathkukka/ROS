# Dual Boot Ubuntu LTS with Windows – Step-by-Step Guide

These are my personal notes for safely setting up a dual-boot system with Ubuntu LTS and Windows. Follow each step carefully to avoid data loss.

---

## 1. Back Up Your Data

- **Backup all important files** from your Windows system to an external drive or cloud storage.
- Create a Windows recovery drive (optional but recommended):
  - Search for "Create a recovery drive" in Windows and follow the prompts.
- Note down your Windows product key (just in case):
  - Open Command Prompt and run:
    ```sh
    wmic path softwarelicensingservice get OA3xOriginalProductKey
    ```

---

## 2. Download Ubuntu LTS ISO

- Go to the [official Ubuntu website](https://ubuntu.com/download/desktop) and download the latest LTS version (e.g., 22.04 LTS).

---

## 3. Create a Bootable USB Drive

- Use a tool like [Rufus](https://rufus.ie/) (Windows) or [balenaEtcher](https://www.balena.io/etcher/) (cross-platform).
- Insert a USB drive (at least 8GB, all data will be erased).
- Open Rufus and select:
  - Device: Your USB drive
  - Boot selection: Ubuntu ISO
  - Partition scheme: **GPT** (for UEFI systems)
  - File system: FAT32
- Click "Start" and wait for completion.

---

## 4. Shrink Windows Partition

- In Windows, open **Disk Management**:
  - Press `Win + X` → "Disk Management"
- Right-click the main Windows partition (usually C:) and select **Shrink Volume**.
- Enter the amount to shrink (at least 30GB recommended for Ubuntu).
- Leave the new space as **Unallocated** (do not format).

---

## 5. Boot from USB

- Insert the bootable USB drive and reboot your computer.
- Enter the BIOS/UEFI menu (usually by pressing `F2`, `F12`, `Esc`, or `Del` during boot).
- Set the USB drive as the first boot device.
- Save and exit BIOS/UEFI.
- Select "Try or Install Ubuntu" from the boot menu.

---

## 6. Install Ubuntu (Partition Setup)

- Start the Ubuntu installer.
- Choose your language and keyboard layout.
- **Installation type:**
  - Select **"Install Ubuntu alongside Windows Boot Manager"** if available (recommended for beginners).
  - If not available, choose **"Something else"** and manually set up partitions:
    - Select the free/unallocated space.
    - Create the following partitions:
      - **Root (`/`)**: ext4, at least 20GB
      - **Swap**: (optional, 2-4GB or equal to RAM if you want hibernation)
      - **Home (`/home`)**: ext4, rest of the space (optional, for personal files)
- **Device for bootloader installation:**
  - Select the main drive (e.g., `/dev/nvme0n1` or `/dev/sda`), NOT a partition like `/dev/sda1`.
- Continue with installation, set your username and password.

---

## 7. Reboot and Set Up GRUB

- When installation finishes, remove the USB drive and reboot.
- You should see the **GRUB boot menu** with options for Ubuntu and Windows.
- If Windows does not appear, boot into Ubuntu and run:
  ```sh
  sudo update-grub
  ```
- If you boot straight into Windows, check BIOS/UEFI boot order and set Ubuntu (or "ubuntu" entry) as first.

---

## 8. Post-Installation Tips

### Update Ubuntu
```sh
sudo apt update && sudo apt upgrade -y
```

### Install Drivers
- Ubuntu usually installs drivers automatically.
- For NVIDIA GPUs, use:
  ```sh
  sudo ubuntu-drivers autoinstall
  ```
- Reboot after installing drivers.

### Set Up a Shared Partition (Optional)
- To share files between Windows and Ubuntu, create a **NTFS partition** during install or use the Windows partition.
- Ubuntu can read/write NTFS out of the box.
- Mount the partition from Ubuntu's file manager or add to `/etc/fstab` for auto-mounting.

### Enable Hibernation (Optional)
- Requires swap partition at least as large as RAM.
- See [Ubuntu Hibernation Guide](https://help.ubuntu.com/community/EnableHibernateWithSwapFile) for details.

### Dual Boot Maintenance
- Always shut down (not hibernate) before switching OSes to avoid file system issues.
- Windows updates may sometimes overwrite GRUB; if so, use a live USB to repair GRUB.

---

## 9. Troubleshooting

- If you don't see the GRUB menu, check BIOS/UEFI boot order.
- If Ubuntu or Windows won't boot, use a live USB to repair bootloaders.
- For more help, check the [Ubuntu Forums](https://ubuntuforums.org/) or [Ask Ubuntu](https://askubuntu.com/).

---

*These notes are for personal reference. Always double-check with official documentation for the latest instructions.*
