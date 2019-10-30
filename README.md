# UNSW MTRN2500 2019 T3 Assignment 2

The assignment specification is available in `assignment2_spec.md`


#### Setup instructions:

1. Open a terminal and navigate to the ROS workspace:  
    a. `ctrl + alt + t`
    b. `cd ~/mtrn2500_ws`      
2. Remove the `old_src` directory (folder):  
     `rm -r old_src`  
3. Set up public key with VM and github account:  
    a. https://help.github.com/en/github/authenticating-to-github/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent  
    b. https://help.github.com/en/github/authenticating-to-github/adding-a-new-ssh-key-to-your-github-account  
4. Clone (copy) your assignment repository:  
    `git clone url` where `url` is your assignment git repository.  
    - The repository url should look like git@github.com:UNSW-MTRN2500/assignment2-YOUR_USER_NAME.git and can be found by clicking the "clone or download" button on the repo home page.

#### Launching the visualisation environment rviz2:  
1. Open a new terminal or another terminal tab  
    `ctrl + alt + t` or `ctrl + shift + T`  
2. Complile:  
    `. mtrn2500_make`  
~3. Run the visualisation node~  
~`ros2 launch assignment2 rviz2.py`~ 
 Previous launch script was not working. 
3. Start rviz2:
    `ros2 run rviz2 rviz2`
4. Config rviz2 by clicking on `file` in rviz2 and select open config. There are two new config file in the launch folder.
    * `rviz_task0_config.rviz` set the global frame to `z0000000/local_frame`, this will show a rectangle marker at the origin. This works does not require working transform broadcast.
    * `rviz_task0_config.rviz` set the global frame to `world_frame`, the provided `TranformBroadcaster` provided node will calculate send the required `world_frame` to `z0000000/local_frame` transform based on `z0000000/pose` message. Working task 2 is required for this.
`

#### Then compile and run assignment2:  
4. Start another terminal tab  
    `ctrl + shift + T`  
5. Complile:  
    `. mtrn2500_make`  
6. Run the assignment2 node (will only do something once you've implemented it)    
    `ros2 run assignment2 assignment2`
