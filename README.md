# sigverse hsr navigation

## How to install

```bash
cd library
pip install .
```

## How to use

```python
import rospy
from geometry_msgs.msg import Pose2D
from sigverse_hsrb_nav import HSRBNavigation
rospy.init_node("hsrb_nav")
hsrbnav = HSRBNavigation()
hsrbnav.navigation(Pose2D(3, 1, 1.57), "abs")
```

## How to create map

```bash
singularity shell -B /run/user/1000,/var/lib/dbus/machine-id --nv env/sandbox_sigverse/
source /entrypoint.sh
source devel/setup.bash
roslaunch sigverse_hsr_nav navigation.launch
```

- teleopeを活用してマップを作成
- 別のターミナルにてmap_saverを利用して保存

```bash
rosrun map_server map_saver -f <保存したいファイル名>
```

## How to run

- mapの表示・確認

```bash
rosrun map_server map_server <map_path>
rosrun tamhome_pointing_estimation visualize_clicked_point_node.py
rviz -d common_pkgs/sigverse_hsrb_nav/config/make_map.rviz
```

- navigationを実行

```bash
roslaunch sigverse_hsr_nav navigation.launch
```
