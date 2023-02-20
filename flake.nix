{
  inputs = {
    std.url = "github:divnix/std";
    std.inputs.nixpkgs.follows = "nix-godot-custom/nixpkgs";
    nix-godot-custom.url = "github:Pylgos/nix-godot-custom";
    nixpkgs.follows = "nix-godot-custom/nixpkgs";
    ros2nix.url = "github:Pylgos/ros2nix";
  };

  outputs = { std, self, ... } @ inputs: std.growOn
    {
      inherit inputs;
      cellsFrom = ./nix;
      cellBlocks = with std.blockTypes; [
        (installables "packages")
        (devshells "devshells")
      ];
    }
    {
      packages = std.harvest self [ "project" "packages" ];
      devShells = std.harvest self [ "project" "devshells" ];
    };
}