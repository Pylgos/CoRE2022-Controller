{ cell, inputs }:
let
  std = inputs.std;
  self = inputs.self;
  nixpkgs = inputs.nixpkgs;
  godot = inputs.nix-godot-custom.godot.packages.godot-camera-branch;
  rosPkgs = inputs.ros2nix.humble.packages;
in {
  default = nixpkgs.mkShell {
    nativeBuildInputs = [
      godot
    ];
  };

  gdextension = nixpkgs.stdenv.mkDerivation {
    pname = "godot-ros";
    version = "0.1.0";

    src = std.incl self [
      (self + "/src")
      (self + "/meson.build")
    ];

    buildInputs = [
      godot.godot-cpp
      rosPkgs.rclcpp
      rosPkgs.sensor_msgs
    ];

    postInstall = ''
      substitute ${./godot-ros.gdextension.in} $out/godot-ros.gdextension \
        --replace @libPath@ $out/lib/libgodot-ros.so
    '';

    nativeBuildInputs = with nixpkgs; [
      pkg-config
      meson
      ninja
    ];
  };
}