
## step 1
to use the nix shell, just install nix from here:
 
https://determinate.systems/nix-installer/
 
and then open a shell within the repo and run `nix develop`

## step 2 
launch the foxglove ros bridge for visualizing the "live" data that will be being broadcast out of the ros2 bag
```ros2 launch foxglove_bridge foxglove_bridge_launch.xml```


## step 3
start playing the ros bag / mcap
```ros2 bag play <bagfile.mcap> --loop```

## step 4
open foxglove studio

## step 5 
run the lidar camera tf pub node

```ros2 run lidar_camera_tf_pub pub```
