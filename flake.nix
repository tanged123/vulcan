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
            janusPackage
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
              gtest
              clang-tools
              doxygen
              graphviz
              lcov
              llvmPackages_latest.llvm
            ]
            ++ [
              janusPackage
              treefmtEval.config.build.wrapper
            ];

          shellHook = ''
            export CMAKE_PREFIX_PATH=${pkgs.eigen}:${pkgs.casadi}:${pkgs.gtest}:${pkgs.hdf5}:${pkgs.highfive}:${janusPackage}
          '';
        };

        formatter = treefmtEval.config.build.wrapper;

        checks = {
          formatting = treefmtEval.config.build.check self;
        };
      }
    );
}
