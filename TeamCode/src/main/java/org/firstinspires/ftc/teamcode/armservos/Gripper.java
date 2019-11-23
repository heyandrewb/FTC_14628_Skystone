package org.firstinspires.ftc.teamcode.armservos;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Gripper {
    CRServo m_gripper;

    TouchSensor m_outLimit;
    TouchSensor m_inLimit;

    public Gripper(CRServo gripper, TouchSensor outLimit, TouchSensor inLimit) {
        m_gripper = gripper;
        m_outLimit = outLimit;
        m_inLimit = inLimit;
    }

    public void open() {
        if(!m_outLimit.isPressed()) {
            m_gripper.setPower(1);
        } else {
            m_gripper.setPower(0);
        }
    }

    public void close() {
        if(!m_inLimit.isPressed()) {
            m_gripper.setPower(-1);
        } else {
            m_gripper.setPower(0);
        }
    }

    public void kill() {
        // TODO: kill
    }
}
