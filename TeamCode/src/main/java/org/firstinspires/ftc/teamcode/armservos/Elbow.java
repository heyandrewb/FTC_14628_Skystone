package org.firstinspires.ftc.teamcode.armservos;

import com.qualcomm.robotcore.hardware.Servo;

public class Elbow {
    Servo m_elbow;

    public Elbow(Servo elbow) {
        m_elbow = elbow;
    }

    public void store() {
        m_elbow.setPosition(-1);
    }

    public void grab() {
        // TODO: grab
    }

    public void score() {
        m_elbow.setPosition(1);
    }
}