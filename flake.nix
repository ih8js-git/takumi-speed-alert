{
  description = "Pi Zero Builds";
  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

  outputs = { self, nixpkgs }: let
    pkgs-aarch64 = import nixpkgs { system = "aarch64-linux"; };
    car-logger = pkgs-aarch64.rustPlatform.buildRustPackage {
      pname = "car-logger";
      version = "0.1.0";
      src = pkgs-aarch64.lib.cleanSource ./.;
      cargoLock = {
        lockFile = ./Cargo.lock;
      };
      cargoBuildFlags = [ "-p" "car-logger" ];
      cargoTestFlags = [ "-p" "car-logger" ];
    };
  in {
    packages.aarch64-linux.car-logger = car-logger;
    nixosConfigurations = {
      
      # Production Build (Lean)
      pizero-prod = nixpkgs.lib.nixosSystem {
        system = "aarch64-linux";
        modules = [ 
          "${nixpkgs}/nixos/modules/installer/sd-card/sd-image-aarch64.nix"
          ./os/configuration.nix 
          ({ ... }: {
            environment.systemPackages = [ car-logger ];
          })
        ];
      };

      # Development Build (Base + Extra Tools)
      pizero-dev = nixpkgs.lib.nixosSystem {
        system = "aarch64-linux";
        modules = [ 
          "${nixpkgs}/nixos/modules/installer/sd-card/sd-image-aarch64.nix"
          ./os/configuration.nix 
          
          # Inject Dev Packages Here
          ({ pkgs, lib, ... }: {
            # Skip slow zstd compression for fast local dev builds
            sdImage.compressImage = false;

            # --- Headless Networking (IWD & Systemd-Networkd) ---
            networking.wireless.enable = lib.mkForce false; 
            networking.wireless.iwd.enable = true;
            
            # Manually provision the open STARLINK network for IWD
            systemd.services.iwd.preStart = ''
              mkdir -p /var/lib/iwd
              cat > /var/lib/iwd/STARLINK.open <<EOF
            [Settings]
            AutoConnect=true
            EOF
            '';

            networking.useDHCP = false;
            systemd.network.enable = true;
            systemd.network.networks."10-wlan" = {
              matchConfig.Name = "wlan0";
              networkConfig.DHCP = "yes";
            };

            # Enable SSH
            services.openssh = {
              enable = true;
              settings.PermitRootLogin = "yes";
              startWhenNeeded = true;
            };
            users.users.root.password = "nixos"; # Change this immediately after login

            # One-shot time sync at boot (replaces timesyncd daemon to save RAM)
            systemd.services.sync-time-once = {
              description = "One-shot time sync";
              after = [ "network.target" "network-online.target" ];
              wants = [ "network-online.target" ];
              wantedBy = [ "multi-user.target" ];
              serviceConfig = {
                Type = "oneshot";
                RemainAfterExit = true;
                TimeoutStartSec = "infinity";
              };
              script = ''
                for i in 1 2 3 4 5; do
                  if ${pkgs.busybox}/bin/ntpd -q -n -p pool.ntp.org; then
                    echo "Time synced successfully."
                    exit 0
                  fi
                  echo "Time sync failed. Retrying in 5 seconds ($i/5)..."
                  sleep 5
                done
                echo "Giving up on time sync (device might be offline)."
                exit 0
              '';
            };

            programs.neovim = {
              enable = true;
              defaultEditor = true;
            };
            environment.systemPackages = [ 
              pkgs.btop 
              pkgs.htop 
              pkgs.git 
              pkgs.fastfetch
              pkgs.tmux
              car-logger
            ];
          })
        ];
      };
    };
  };
}
