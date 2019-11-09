package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Lift {
    DcMotor m_liftMotor;

    public Lift (DcMotor liftMotor)
    {
        m_liftMotor = liftMotor;
    }

    public void raiseLift (double speed) {
        m_liftMotor.setPower(Math.abs(speed));
    }

    public void lowerLift (double speed) {
        m_liftMotor.setPower(-1 * Math.abs(speed));
    }
}
