package org.firstinspires.ftc.teamcode.armservos;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Gripper {
    Servo m_gripper;

    TouchSensor m_outLimit;
    TouchSensor m_inLimit;

    public Gripper(Servo gripper, TouchSensor outLimit, TouchSensor inLimit) {
        m_gripper = gripper;
        m_outLimit = outLimit;
        m_inLimit = inLimit;
    }

    public void open() {
        // TODO: open
    }

    public void close() {
        // TODO: close
    }

    public void kill() {
        // TODO: kill
    }
}
