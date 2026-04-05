{
  description = "Vulcan: Aerospace Engineering Utilities";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    treefmt-nix.url = "github:numtide/treefmt-nix";

    # Janus as a flake input
    janus = {
      url = "github:tanged123/janus";
      # Or for local development:
      # url = "path:/home/tanged/sources/janus";
    };
  };

  outputs =
    {
      self,
      nixpkgs,
      flake-utils,
      treefmt-nix,
      janus,
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = nixpkgs.legacyPackages.${system};
        stdenv = pkgs.llvmPackages_latest.stdenv;

        # Get janus package from input
        janusPackage = janus.packages.${system}.default;

        # mp-units: C++20 physical units library (not in nixpkgs)
        mp-units = stdenv.mkDerivation {
          pname = "mp-units";
          version = "2.5.0";

          src = pkgs.fetchFromGitHub {
            owner = "mpusz";
            repo = "mp-units";
            rev = "v2.5.0";
            hash = "sha256-HP5eq5NJIAsK3HuTwIvJgt5Y3gSMpmOng+/HDXBv0ZA=";
          };

          nativeBuildInputs = [
            pkgs.cmake
            pkgs.ninja
          ];

          buildInputs = [
            pkgs.fmt
          ];

          cmakeFlags = [
            "-DMP_UNITS_API_CONTRACTS=NONE"
          ];

          # The CMakeLists.txt is in src/ subdirectory
          sourceRoot = "source/src";
        };

        # Treefmt configuration
        treefmtEval = treefmt-nix.lib.evalModule pkgs {
          projectRootFile = "flake.nix";
          programs.nixfmt.enable = true;
          programs.clang-format.enable = true;
          programs.cmake-format.enable = true;
        };
      in
      {
        packages.default = stdenv.mkDerivation {
          pname = "vulcan";
          version = "0.4.0";
          src = ./.;

          nativeBuildInputs = [
            pkgs.cmake
            pkgs.ninja
            pkgs.pkg-config
          ];

          buildInputs = [
            pkgs.eigen
            pkgs.casadi
            pkgs.hdf5
            pkgs.highfive # C++ HDF5 wrapper
            pkgs.yaml-cpp
            janusPackage
            mp-units
          ];

          cmakeFlags = [
            "-DENABLE_COVERAGE=OFF"
            "-DBUILD_TESTING=OFF"
            "-DBUILD_EXAMPLES=OFF"
          ];
        };

        devShells.default = pkgs.mkShell.override { inherit stdenv; } {
          packages =
            with pkgs;
            [
              cmake
              ninja
              pkg-config
              ccache # For compiler caching
              eigen
              casadi
              hdf5
              highfive
              yaml-cpp
              gtest
              clang-tools
              doxygen
              graphviz
              lcov
              llvmPackages_latest.llvm
              fmt
            ]
            ++ [
              janusPackage
              mp-units
              treefmtEval.config.build.wrapper
            ];

          shellHook = ''
            export CMAKE_PREFIX_PATH=${pkgs.eigen}:${pkgs.casadi}:${pkgs.gtest}:${pkgs.hdf5}:${pkgs.highfive}:${pkgs.yaml-cpp}:${pkgs.fmt}:${janusPackage}:${mp-units}
          '';
        };

        formatter = treefmtEval.config.build.wrapper;

        checks = {
          formatting = treefmtEval.config.build.check self;
        };
      }
    );
}
