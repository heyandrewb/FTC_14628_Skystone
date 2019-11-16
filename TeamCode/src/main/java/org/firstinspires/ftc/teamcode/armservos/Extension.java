package org.firstinspires.ftc.teamcode.armservos;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Extension {
    Servo m_extension;

    TouchSensor m_outLimit;
    TouchSensor m_inLimit;

    public Extension(Servo extension, TouchSensor outLimit, TouchSensor inLimit) {
        m_extension = extension;
        m_outLimit = outLimit;
        m_inLimit = inLimit;
    }

    public void open() {
        // TODO: open
    }

    public void close() {
        // TODO: close
    }

    public void moveManualOut() {
        // TODO: moveManualOut
    }

    public void moveManualIn() {
        // TODO: moveManualInt
    }

    public void kill() {
        // TODO: kill
    }
}
