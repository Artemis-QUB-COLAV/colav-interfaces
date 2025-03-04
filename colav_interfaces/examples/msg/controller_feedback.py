from colav_interfaces.msg import ControllerFeedback

def controller_feedback_example() -> ControllerFeedback:
    controller_feedback = ControllerFeedback()
    controller_feedback.mission_tag = "mission_0001"
    controller_feedback.agent_tag = "agent_0001"
    controller_feedback.header.stamp.sec = 1000
    controller_feedback.header.stamp.nanosec = 20030
    controller_feedback.header.frame_id = "map"
    
    controller_feedback.cmd_vel_yaw.velocity =  10.0
    controller_feedback.cmd_vel_yaw.yaw_rate = 0.2
    
    controller_feedback.ctrl_mode.control_mode = 3
    
    controller_feedback.ctrl_status.status = 1
    controller_feedback.ctrl_status.message = "success"
    return controller_feedback

def main():
    controller_feedback = controller_feedback_example()
    print (controller_feedback)

if __name__ == '__main__':
    main()