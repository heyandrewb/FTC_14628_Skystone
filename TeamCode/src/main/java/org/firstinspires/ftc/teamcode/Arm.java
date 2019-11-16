package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.armservos.Elbow;
import org.firstinspires.ftc.teamcode.armservos.Extension;
import org.firstinspires.ftc.teamcode.armservos.Gripper;

public class Arm {
    Elbow m_armElbow;
    Extension m_armExtension;
    Gripper m_armGripper;

    public Arm(SkystoneHardwareMap robot) {
        m_armElbow = new Elbow(robot.elbowServo);
        m_armExtension = new Extension(robot.extensionServo, robot.extensionLimitOut, robot.extensionLimitIn);
        m_armGripper = new Gripper(robot.gripperServo, robot.gripperLimitOut, robot.gripperLimitIn);
    }
}