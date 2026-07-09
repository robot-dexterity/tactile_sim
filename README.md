# Tactile-Sim: Simulation of various robot arms and tactile Sensors
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](LICENSE)

<!-- [Project Website](https://sites.google.com/my.bristol.ac.uk/tactile-gym-sim2real/home) &nbsp;&nbsp;• -->
**Tactile Gym 2.0**: [Project Website](https://sites.google.com/my.bristol.ac.uk/tactilegym2/home) &nbsp;&nbsp;•&nbsp;&nbsp;[Paper](https://ieeexplore.ieee.org/abstract/document/9847020)

**Tactile Gym 1.0**: [Project Website](https://sites.google.com/my.bristol.ac.uk/tactile-gym-sim2real/home) &nbsp;&nbsp;•&nbsp;&nbsp;[Paper](http://arxiv.org/abs/2106.08796)

This repo contains code for the simulation of various robot arms with tactile sensors in the PyBullet physics engine. Currently, 4 robot arms (UR5, Franka-Panda, Kuke-iiwa and Dobot CR3) are implemented along with tactile sensors of the [Tactip](https://www.liebertpub.com/doi/full/10.1089/soro.2017.0052) and [Gelsight](https://www.mdpi.com/1424-8220/17/12/2762) style.


<p align="center">
  <img src="docs/readme_videos/surf_arm_transfer.gif">
</p>


### Content ###
- [Installation](#installation)
- [Robot Arms](#robot-arms)
- [Tactile Sensors](#tactile-sensors)
- [Visual Sensors](#visual-sensors)
- [Embodiments](#embodiments)
- [Control](#control)
- [Additional Info](#additional-info)



### Installation ###
This repo has only been developed and tested with Ubuntu 18.04 and python 3.8.

```console
git clone https://github.com/dexterousrobot/tactile_sim
cd tactile_sim
python setup.py install
```

Once installed, move the resulting tactile_sim directory somewhere python will search for packages. You can see suitable locations by running a python file with:
```
import sys
print(sys.path)
```


Demonstration files are provided for in the example directory. From the base directory run
```
python examples/demo_robot_control.py
```
to run a user controllable example environment.


### Robot Arms ###

- `UR5`: 6 DoF robot arm from Universal Robotics.
- `Franka-Panda`: 7 DoF robot arm from Franka-Emika
- `Kuka-IIWA`: 7 DoF robot arm form KUKA Robotics.
- `Dobot-CR3`: 6 DoF robot arm from DoBot.
- `Dobot-MG400`: Prelimanary support for the 4DoF, Dobot MG400. Due to the limited degrees of freedom this arm cannot be easily interchanged with the others.

| **Parameters** | Description | Options | 
| :---: | :--- |:--- |
| `type` | Which robot arm to use. | `ur5` `franka_panda` `kuka_iiwa` `cr3` `mg400`|
| `rest_poses` | Numpy array of default joint positions initiated when the robot arm is reset. | See `tactile_sim.assets.default_rest_poses` for examples. |



### Tactile Sensors ###


- `TacTip`: [Tactip](https://www.liebertpub.com/doi/full/10.1089/soro.2017.0052) is a soft, curved, 3D-printed tactile skin with an internal array of pins tipped with markers, which are used to amplify the surface deformation from physical contact against a stimulus.
- `DIGIT`: [DIGIT](https://digit.ml/) shares the same principle of the [Gelsight tactile sensor](https://www.mdpi.com/1424-8220/17/12/2762), but can be fabricated at low cost and is of a size suitable for integration of some robotic hands, such as on the fingertips of the Allegro.
- `DigiTac`: DigiTac is an adapted version of the DIGIT and the TacTip, whereby the 3D-printed skin of a TacTip is customized to integrated onto the DIGIT housing, while keeping the camera and lighting system. In other words, this sensor outputs tactile images of the same dimension as the DIGIT, but with a soft biomimetic skin like other TacTip sensors.

| **Parameters** | Description | Options | 
| :---: | :--- |:--- |
| `type` | Which tactile sensor to use. | `standard_tactip` `standard_digit` `standard_digitac` `mini_tactip` `flat_tactip` `right_angle_tactip` `right_angle_digit` `right_angle_digitac`|
| `core` | The type of internal "gel" to use in simulation. | `no_core` `fixed` |
| `dynamics` | If fixed core is specified then this gives dynamics params for the contact. | e.g. `{'stiffness': 50, 'damping': 100, 'friction': 10.0}` |
| `image_size` | Dimensions of the tactile image to be rendered. | e.g. `[128, 128]` |
| `turn_off_border` | Flag for disabling borders within the tactile image. | `True` `False` |
| `show_tactile` | Flag for enabling visualisation of tactile images. | `True` `False` |


### Vision Sensors ###

- `Camera`: Visual camera as implemented in PyBullet. Able to return an RGBA image, Depth image and Segmentaion image.

| **Parameters** | Description | Options | 
| :---: | :--- |:--- |
| `image_size` | Dimensions of the tactile image to be rendered. | e.g. `[128, 128]` |
| `dist` | Distance of camera from focal point. | e.g. `1.0` |
| `yaw` | Yaw angle of camera. | e.g. `90.0` |
| `pitch` | Pitch angle of camera. | e.g. `-25.0` |
| `pos` | Position of point camera is focused on. | e.g. `[0.6, 0.0, 0.0525]` |
| `fov` | Field of view | e.g. `75.0` |
| `near_val` | Near plane distance. | e.g. `0.1` |
| `far_val` | Far plane distance. | e.g. `100.0` |
| `show_visual` | Flag for enabling visualisation of visual images. | `True` `False` |

### Embodiments ###

- `ArmEmbodiment`: No sensors, just control of a robot arm. 
- `TactileArmEmbodiment`: Tactile sensor attached as the EE of a robot arm. Inherits control from ArmEmbodiment.
- `VisualArmEmbodiment`: Static visual sensor within the environment. Inherits control from ArmEmbodiment. 
- `VisuotactileArmEmbodiment`: Combination of VisualArmEmbodiment and TactileArmEmbodiment. 


### Control ###
| **Control Mode** | Description | 
| :---: | :--- |
| `tcp_position_control` | Move the tool center point to a target position in the world frame, block until position reached of max_steps. |
| `tcp_veloicty_control` | Move the tool center point at a target velocity, block for fixed number of steps. | 
| `joint_position_control` | Move the joints to target positions, block until targets reached of max_steps. |
| `joint_veloicty_control` | Move the joints at a target velocity, block for fixed number of steps. | 


### Bibtex ###

If you use this repo in your work, please cite

```
@InProceedings{lin2022tactilegym2,
     title={Tactile Gym 2.0: Sim-to-real Deep Reinforcement Learning for Comparing Low-cost High-Resolution Robot Touch},
     author={Yijiong Lin and John Lloyd and Alex Church and Nathan F. Lepora},
     journal={IEEE Robotics and Automation Letters},
     year={2022},
     volume={7},
     number={4},
     pages={10754-10761},
     editor={R. Liu A.Banerjee},
     series={Proceedings of Machine Learning Research},
     month={August},
     publisher={IEEE},
     doi={10.1109/LRA.2022.3195195}}
     url={https://ieeexplore.ieee.org/abstract/document/9847020},
}

@InProceedings{church2021optical,
     title={Tactile Sim-to-Real Policy Transfer via Real-to-Sim Image Translation},
     author={Church, Alex and Lloyd, John and Hadsell, Raia and Lepora, Nathan F.},
     booktitle={Proceedings of the 5th Conference on Robot Learning}, 
     year={2022},
     editor={Faust, Aleksandra and Hsu, David and Neumann, Gerhard},
     volume={164},
     series={Proceedings of Machine Learning Research},
     month={08--11 Nov},
     publisher={PMLR},
     pdf={https://proceedings.mlr.press/v164/church22a/church22a.pdf},
     url={https://proceedings.mlr.press/v164/church22a.html},
}
```
