#!/usr/bin/env python3
#coding: utf-8

import rclpy
import yasmin

from yasmin import StateMachine, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL, TIMEOUT

from state_machine.machines import FollowMeMachine ,NavigateToTargetMachine ,SaySomethingMachine
from state_machine.states import ComputeTargetFromPoseState ,TrackedPersonPoseState ,InterbotixTargetPose


def main():
    yasmin.YASMIN_LOG_INFO("Testing WaitTimeState")
    
    rclpy.init()
    # set_ros_loggers()

    blackboard = Blackboard()
    
    sm = StateMachine(outcomes=[SUCCEED, ABORT, CANCEL, TIMEOUT])
    
    blackboard["number"] = 3

    sm.add_state("SAYSOMETHING_MACHINE", SaySomethingMachine(data: str = "I'm Ready", lang: str = "en", use_topic: bool = True), transitions={
        SUCCEED: SUCCEED,
        ABORT: ABORT,
        CANCEL: CANCEL,
        TIMEOUT: TIMEOUT,
    })

    sm.add_state("TRACK_PERSON", TrackedPersonPoseState(self, topic: str, 
               timeout: float = 2, 
               max_distance: float = 6, 
               mapped_param: bool = True), transitions={
        SUCCEED: SUCCEED,
        ABORT: ABORT,
        CANCEL: CANCEL,
        TIMEOUT: TIMEOUT,
    })


    sm.add_state("COMPUTE_TARGET", ComputeTargetFromPoseState(self, offset: float = 0.0, 
               fixed_frame: str = 'map',
               robot_frame: str = 'base_link', 
               use_pose_stamp: bool = False, 
               timeout: float=1.0), transitions={
        SUCCEED: SUCCEED,
        ABORT: ABORT,
        CANCEL: CANCEL,
        TIMEOUT: TIMEOUT,
    })

    sm.add_state("FOLLOW_TARGET", FollowMeMachine(self,
                 tracking_topic: str = '/fbot_vision/pt/tracking3D'), transitions={
        SUCCEED: SUCCEED,
        ABORT: ABORT,
        CANCEL: CANCEL,
        TIMEOUT: TIMEOUT,
    })

    sm.add_state("GRAB_TARGET", InterbotixTargetPose(self,
            arm_model: str = 'wx200',
            target_joint: str = None,
            target_pose: str = None), transitions={
        SUCCEED: SUCCEED,
        ABORT: ABORT,
        CANCEL: CANCEL,
        TIMEOUT: TIMEOUT,
    })

    sm.add_state("NAVIGATE_TARGET", NavigateToTargetMachine(self,
                 key : list = None), transitions={
        SUCCEED: SUCCEED,
        ABORT: ABORT,
        CANCEL: CANCEL,
        TIMEOUT: TIMEOUT,
    })

    try:
        outcome = sm(blackboard)
        yasmin.YASMIN_LOG_INFO(f"State machine finished with outcome: {outcome}")
    except KeyboardInterrupt:
        if sm.is_running():
            sm.cancel_state()
    
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == "__main__":
    main()