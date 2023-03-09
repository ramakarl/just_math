Quick Start
===

Example usage of `img2tile` for mario and pacman:

```
$ ./img2tile -i demo_pacman.png -s 8 -w 16 -S pm_tileset.png -N pm_tilename.csv -R pm_tilerule.csv -t 0
$ ./img2tile -i demo_mario.png -s 16 -w 32 -S smb_tileset.png -N smb_tilename.csv -R smb_tilerule.csv
```

Example usage of mario for a BPC run:

```
$ ./bpc -N smb_tilename.csv -R smb_tilerule.csv -I 100000 -X 24 -Y 12 -Z 1 -S 0 -G 2 -M smb_tiled.json -Q ./smb_tileset.png -s 16 -c 0 -V 2
$ ./bpc -N pm_tilename.csv -R pm_tilerule.csv -I 100000 -X 16 -Y 18 -Z 1 -S 1 -G 2 -M pm_tiled.json -Q ./pm_tileset.png -s 8 -c 0 -V 2
```
