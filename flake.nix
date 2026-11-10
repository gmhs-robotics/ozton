{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

    flake-utils.url = "github:numtide/flake-utils";
    cargo-v5 = {
      url = "github:gmhs-robotics/cargo-v5/main";
      flake = false;
    };
    vex-v5-serial = {
      url = "github:vexide/vex-v5-serial";
      flake = false;
    };
    rust-overlay.url = "github:oxalica/rust-overlay";
  };

  outputs =
    {
      nixpkgs,
      flake-utils,
      cargo-v5,
      vex-v5-serial,
      rust-overlay,
      ...
    }:
    (flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ (import rust-overlay) ];
        };
        # cargo-v5 expects vex-v5-serial in a sibling checkout.
        cargoV5 = pkgs.rustPlatform.buildRustPackage {
          pname = "cargo-v5";
          inherit ((builtins.fromTOML (builtins.readFile "${cargo-v5}/Cargo.toml")).package) version;
          src = pkgs.runCommand "cargo-v5-source" { } ''
            mkdir -p "$out"
            cp -R ${cargo-v5} "$out/cargo-v5"
            cp -R ${vex-v5-serial} "$out/vex-v5-serial"
          '';
          cargoRoot = "cargo-v5";
          buildAndTestSubdir = "cargo-v5";
          cargoLock.lockFile = "${cargo-v5}/Cargo.lock";
          nativeBuildInputs = [ pkgs.pkg-config ];
          buildInputs = [ pkgs.udev ];
        };
        rustToolchain = pkgs.rust-bin.fromRustupToolchainFile ./rust-toolchain.toml;
      in
      {
        devShells.default = pkgs.mkShell {
          buildInputs = [
            pkgs.ripgrep
            pkgs.mdbook
            pkgs.cargo-release
            cargoV5
            (rustToolchain.override {
              extensions = [
                "rust-analyzer"
                "rust-src"
                "clippy"
              ];
            })
          ];
        };
      }
    ));
}
