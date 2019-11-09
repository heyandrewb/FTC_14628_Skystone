package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class mecanumDrive {

    public static void drive(double forward, double strafe, double rotate, DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {

        frontLeft.setPower ( forward - strafe + rotate);
        frontRight.setPower( forward + strafe - rotate);
        backLeft.setPower  ( forward + strafe + rotate);
        backRight.setPower ( forward - strafe - rotate);

    }
}
