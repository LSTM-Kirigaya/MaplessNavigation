# MaplessNavigation

Reinforcement learning algorithm for mapless navigation

- `env`: environment for RL train and test

- `src`: source of the RL algorithm

- `robot`: urdf and functions to operate robots   

---

Use pybullet to build env RL algorithm needs. Robot used in the project is `miniBox`, which is described in `.\robot\miniBox.urdf`.

`miniBox` is a differentially dirven robot.

<center>
    <img style="border-radius: 0.3125em;
    box-shadow: 0 2px 4px 0 rgba(34,36,38,.12),0 2px 10px 0 rgba(34,36,38,.08);" 
    src="https://i.loli.net/2021/02/10/HjoJK87T52YMOxa.png" width = "80%" alt=""/>
    <br>
</center>

Functions regarding to operating robot and create environmental items are implemented in `.\robot\utils.py`

