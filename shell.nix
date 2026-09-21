{ pkgs ? import <nixpkgs> {} }:

pkgs.mkShell {
  buildInputs = with pkgs; [
    stdenv.cc.cc.lib
    zlib
    glib
    libglvnd
    libGL
    xorg.libxcb
    xorg.libX11
    xorg.libXext
    xorg.libXrender
    xorg.libICE
    xorg.libSM
  ];

  shellHook = ''
    export LD_LIBRARY_PATH="${pkgs.lib.makeLibraryPath [
      pkgs.stdenv.cc.cc.lib
      pkgs.zlib
      pkgs.glib
      pkgs.libglvnd
      pkgs.libGL
      pkgs.xorg.libxcb
      pkgs.xorg.libX11
      pkgs.xorg.libXext
      pkgs.xorg.libXrender
      pkgs.xorg.libICE
      pkgs.xorg.libSM
    ]}:$LD_LIBRARY_PATH"
  '';
}