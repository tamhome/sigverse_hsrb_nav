# sigverse hsr navigation

## How to install

```bash
cd library
pip install .
```

## How to use

```python
from sigverse_hsr_nav import HSRBNavigation
hsrnav = HSRBNavigation()
hsrnav.navigation(Pose2D(0, 0, 0), "abs")
```

## How to run

```bash
roslaunch sigverse_hsr_nav navigation.launch
```
