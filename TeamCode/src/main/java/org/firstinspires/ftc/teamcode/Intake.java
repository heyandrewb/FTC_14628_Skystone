package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    Servo m_intakeLeftServo;
    Servo m_intakeRightServo;

    DcMotor m_intakeLeftWheel;
    DcMotor m_intakeRightWheel;

    final double down = 0.0;
    final double shoot = 0.2;
    final double store = 0.4;

    final double wheelInSpeed = 0.7;
    final double wheelOutSpeed = 0.3;
    final double shootSpeed = 1.0;

    public Intake(Servo intakeLeftS, Servo intakeRightS, DcMotor intakeLeftWheel, DcMotor intakeRightWheel) {
        m_intakeLeftServo = intakeLeftS;
        m_intakeRightServo = intakeRightS;

        m_intakeLeftWheel = intakeLeftWheel;
        m_intakeRightWheel = intakeRightWheel;
    }

    public void wheelIn() {
        m_intakeLeftWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        m_intakeRightWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        m_intakeLeftWheel.setPower(wheelInSpeed);
        m_intakeRightWheel.setPower(wheelInSpeed);
    }

    public void wheelShoot() {
        m_intakeLeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        m_intakeRightWheel.setDirection(DcMotorSimple.Direction.FORWARD);

        m_intakeLeftWheel.setPower(shootSpeed);
        m_intakeRightWheel.setPower(shootSpeed);
    }

    public void wheelOutSlow() {
        m_intakeLeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        m_intakeRightWheel.setDirection(DcMotorSimple.Direction.FORWARD);

        m_intakeLeftWheel.setPower(wheelOutSpeed);
        m_intakeRightWheel.setPower(wheelOutSpeed);
    }

    public void wheelKill() {
        m_intakeLeftWheel.setPower(0.0);
        m_intakeLeftWheel.setPower(0.0);
    }

    public void store() {
        m_intakeLeftServo.setPosition(store);
        m_intakeRightServo.setPosition(store);
    }

    public void shootRaise() {
        m_intakeLeftServo.setPosition(shoot);
        m_intakeRightServo.setPosition(shoot);
    }

    public void lower() {
        m_intakeLeftServo.setPosition(down);
        m_intakeRightServo.setPosition(down);
    }
}