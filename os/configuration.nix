{ config, pkgs, lib, modulesPath, ... }:

{
  imports = [
    (modulesPath + "/profiles/minimal.nix")
  ];
  system.stateVersion = "24.05";
  networking.hostName = "pizero";


  # Build Speed & Disk Space Optimizations
  sdImage.firmwareSize = 2048; # 2GB FAT32 partition for user data drops
  hardware.enableRedistributableFirmware = lib.mkForce false;
  hardware.firmware = [ pkgs.raspberrypiWirelessFirmware ];
  sdImage.postBuildCommands = ''
    ${pkgs.bmaptool}/bin/bmaptool create $img -o $img.bmap
  '';

  # Hardware Tweaks (Reclaim GPU RAM and Kill Bluetooth)
  sdImage.populateFirmwareCommands = lib.mkAfter ''
    chmod +w firmware/config.txt
    echo "gpu_mem=16" >> firmware/config.txt
    echo "dtoverlay=disable-bt" >> firmware/config.txt
  '';

  # Blacklist Kernel Modules
  boot.blacklistedKernelModules = [ 
    # Kill Bluetooth completely
    "bluetooth" 
    "btusb" 
    "hci_uart"
    "btbcm"
    "btqca"

    # Kill Camera / Video4Linux
    "videodev"
    "videobuf2_v4l2"
    "videobuf2_vmalloc"
    "videobuf2_common"
    "videobuf2_memops"
    "bcm2835_v4l2"
    "bcm2835_mmal_vchiq"

    # Kill HDMI / Display (if you never plug in a monitor)
    "vc4"
    "drm_display_helper"
    "drm_dma_helper"
    "drm_exec"
    "cec"
  ];

  # Aggressive Bloat Removal
  documentation.enable = false;
  documentation.doc.enable = false;
  documentation.man.enable = false;
  documentation.info.enable = false;
  services.udisks2.enable = false;
  environment.defaultPackages = [];
  
  # Set up system packages
  environment.systemPackages = with pkgs; [ ];

  # Extreme Memory Optimizations
  boot.kernelParams = [ "cgroup_disable=memory" "ipv6.disable=1" "audit=0" ];
  security.polkit.enable = false;
  networking.firewall.enable = false;
  services.journald.extraConfig = "SystemMaxUse=256M\nRuntimeMaxUse=8M";
  services.resolved.enable = false;
  services.timesyncd.enable = false;
  systemd.oomd.enable = false;
  systemd.services."getty@tty1".enable = false;
  systemd.services."autovt@tty1".enable = false;

  # Mount the FAT32 boot partition so we can access state_bins and write car_logs
  fileSystems."/boot/firmware" = {
    device = "/dev/disk/by-label/FIRMWARE";
    fsType = "vfat";
    options = lib.mkForce [ "rw" "umask=000" "nofail" "auto" "x-systemd.device-timeout=5s" ];
  };
  
  # Strip ZFS and BTRFS
  boot.supportedFilesystems = lib.mkForce [ "vfat" "ext4" ];
  boot.initrd.services.lvm.enable = false;
  boot.swraid.enable = lib.mkForce false;

  # Prevent Nix from keeping excessive build generations locally
  nix.settings.auto-optimise-store = true;
  nix.gc = {
    automatic = true;
    dates = "weekly";
    options = "--delete-older-than 7d";
  };
}
