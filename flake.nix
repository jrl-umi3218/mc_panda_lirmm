{
  description = "mc-panda-lirmm: flakoboros and mc-rtc-superbuild shell for mc-panda-lirmm robot module";

  inputs = {
    mc-rtc-nix.url = "github:mc-rtc/nixpkgs/pull/64/head";
    flake-parts.follows = "mc-rtc-nix/flake-parts";
    systems.follows = "mc-rtc-nix/systems";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } (
      { lib, ... }:
      {
        systems = import inputs.systems;
        imports = [
          inputs.mc-rtc-nix.flakeModule
          {
            mc-rtc-superbuild =
              { pkgs, ... }:
              {
                enable = true;
                project.pname = "";
                configurations = {
                  mc-panda-lirmm-minimal = {
                    extends = [ "minimal" ];
                    runtime.apps = [ pkgs.mc-rtc-magnum ];
                    devel.robots = [
                      pkgs.mc-panda
                      pkgs.mc-panda-lirmm
                    ];
                  };
                };
              };
            flakoboros = {
              overrideAttrs.mc-panda-lirmm = {
                src = lib.cleanSource ./.;
              };
            };
          }
        ];
      }
    );
}
