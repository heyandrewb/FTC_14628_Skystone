package org.firstinspires.ftc.teamcode.armservos;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Extension {
    CRServo m_extension;

    TouchSensor m_outLimit;
    TouchSensor m_inLimit;

    public Extension(CRServo extension, TouchSensor outLimit, TouchSensor inLimit) {
        m_extension = extension;
        m_outLimit = outLimit;
        m_inLimit = inLimit;
    }

    public void open() {
        if(!m_outLimit.isPressed()) {
            m_extension.setPower(1);
        } else {
            m_extension.setPower(0);
        }
    }

    public void close() {
        if(!m_inLimit.isPressed()) {
            m_extension.setPower(-1);
        } else {
            m_extension.setPower(0);
        }
    }

    public void moveManualOut() {
        // TODO: moveManualOut
    }

    public void moveManualIn() {
        // TODO: moveManualInt
    }

    public void kill() {
        m_extension.setPower(0);
    }
}
