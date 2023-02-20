{ inputs
, cell
}:
let
  ros2nix = inputs.ros2nix;
  rosPkgs = ros2nix.legacyPackages.humble;
  nixpkgs = inputs.nixpkgs;
  godot = inputs.nix-godot-custom.godot.packages.godot-camera-branch;
  l = inputs.nixpkgs.lib // builtins;
in
{
  default = nixpkgs.mkShell {
    nativeBuildInputs = with nixpkgs; [
      godot
      pkg-config
      meson
      ninja
    ];


    buildInputs = [
      godot.godot-cpp
      rosPkgs.rclcpp
      rosPkgs.sensor_msgs
    ];
  };
}
