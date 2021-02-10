# MaplessNavigation

Reinforcement learning algorithm for mapless navigation

- `env`: environment for RL train and test

- `src`: source of the RL algorithm

- `robot`: urdf and functions to operate robots   

---

Use pybullet to build env RL algorithm needs. Mainly used robot in the project is `miniBox`, which described in `.\robot\miniBox.urdf`.

`miniBox` is a differentially dirven robot.

<center>
    <img style="border-radius: 0.3125em;
    box-shadow: 0 2px 4px 0 rgba(34,36,38,.12),0 2px 10px 0 rgba(34,36,38,.08);" 
    src="https://i.loli.net/2021/02/10/HjoJK87T52YMOxa.png" width = "80%" alt=""/>
    <br>
    <div style="color:orange; border-bottom: 1px solid #d9d9d9;
    display: inline-block;
    color: #999;
    padding: 2px;">
      miniBox preview
  	</div>
</center>

Functions about operating robot and create environmental items are implemented in `.\robot\test_robot.py`
