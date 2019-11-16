package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Lift {
    DcMotor m_liftMotor;
    TouchSensor m_downLimit;
    TouchSensor m_upLimit;


    public Lift (DcMotor liftMotor, TouchSensor downLimit, TouchSensor upLimit) {
        m_liftMotor = liftMotor;
        m_downLimit = downLimit;
        m_upLimit = upLimit;
    }

    public void raiseLift (double speed) {
        m_liftMotor.setPower(Math.abs(speed));
    }

    public void lowerLift (double speed) {
        m_liftMotor.setPower(-1 * Math.abs(speed));
    }

    public void hold() {
        m_liftMotor.setPower(0.0);
    }

    public void store() {
        // TODO: store
    }

    public void kill() {
        m_liftMotor.setPower(0.0);
    }
}
