# GUI functions/descriptions

Some of the stuff we're currently working on!

## arm_gui_controller.py
- Code to grey out buttons based on mode was implemented. Will likely need to be updated as more modes are implemented
- Edit the speed_multiplier variable to change how fast the arm moves
    - 6900 seems to work for manual mode
    - IK needs a much smaller value (69 is the value used)
    - For joint 4, due to its type being int8 and its values being constrained, a for loop is used to update when pressed (WIP because, even though it works, it is not best practice)
- All the button functions work in reverse (ie. forwards moves backwards) in manual mode (but works properly in IK mode)
- We can control some of the arm's individual joints with the individual buttons 

## qt6_gui.py
- Sometimes seems to seg fault when RViz is closed (WIP)

### in __init__:
- The topics in the camera_topic_name list need to be edited to work with actual robot

### in update_end_effector_coords:
- Publisher from Parm's code, called arm_end_effector_pos, within arm_inverse_kinematics.py

### in initUI:
- None of the open/close grip buttons work yet (WIP)
- Move to home/move to origin buttons not functional (WIP)
- None of the ON/OFF buttons functional (WIP)