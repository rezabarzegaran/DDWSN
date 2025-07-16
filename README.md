# DDWSN
Dynamic Deployment of Heterogeneous Wireless Sensor Drone Networks with Limited Communication Range.


This code is used for the paper titiled:
"Dynamic Deployment of Heterogeneous Wireless Sensor Drone Networks with Limited Communication Range"

Published in IEEE Transactions on Vehicular Technology.

The code is developed by Mohammadreza (Reza) Barzegaran (www.barzegaran.xyz) as a part of his postdoc as University of California Irvine, Center for Pervasive Communications and Computing.

Please use Matlab 2022 or earlier and refer to the paper to understand the code and modify the parameters. The code optimizes the deployment for a swarm of drones. It uses the sensing performance C as described in the paper. The drone trajectories are modeled using Bezier curves and predicted using MPC. Then the constraints are applied to the trajectories using quadratic programming. THe constraints are:

Drone Dynamic behavior
Drone velocity and acceleration
Collision avoidance on drones and obstacles.
Domain constraints such as max alt.
Connectivity constraints to keep the drone networks connected and sparce.
Any questions, contact Reza Barzegaran. Feel free to use the code and do not forget to cite the paper as:

@article{BarzegaranTVT2025,
  title={Dynamic Deployment of Heterogeneous Wireless Sensor Drone Networks with Limited Communication Range},
  author={Barzegaran, Mohammadreza and Jafarkhani, Hamid},
  journal={IEEE Transactions on Vehicular Technology},
  volume={},
  number={},
  pages={Accepted for publication},
  year={2025},
  publisher={}
}
