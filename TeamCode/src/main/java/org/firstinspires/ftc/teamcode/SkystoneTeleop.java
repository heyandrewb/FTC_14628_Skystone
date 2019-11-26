package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.utilFuncs;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "Skystone Teleop", group = "Skystone")
//@Disabled //---  Uncomment to disable this teleop mode
public class SkystoneTeleop extends OpMode {

    /* Declare OpMode members. */
    SkystoneHardwareMap robot = new SkystoneHardwareMap();

    public double forward = 0.0;
    public double strafe = 0.0;
    public double rotate = 0.0;

    public double liftPower = 0.0;

    Lift m_lift;
    Intake m_intake;
    Arm m_arm;

    Sensors m_sensors;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        robot.frontLeftDrive.setPower(0.0);
        robot.frontRightDrive.setPower(0.0);
        robot.backLeftDrive.setPower(0.0);
        robot.backRightDrive.setPower(0.0);

        m_lift = new Lift(robot.liftMotor, robot.liftDownLimit, robot.liftUpLimit);
        m_intake = new Intake(robot.leftIntakeServo, robot.rightIntakeServo, robot.leftIntakeWheel, robot.rightIntakeWheel);
        m_arm = new Arm(robot);

        m_sensors = new Sensors(robot.colorSensor, robot.distLeft, robot.distRight);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Waiting for start...", "");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        forward = Range.clip(gamepad1.left_stick_y, -1, 1);
        strafe  = Range.clip(gamepad1.left_stick_x, -1, 1);
        rotate  = Range.clip(gamepad1.right_stick_x, -1, 1);

        liftPower = Range.clip(gamepad2.left_stick_y, -1, 1);

        mecanumDrive.drive(forward, strafe, rotate, robot.frontLeftDrive, robot.frontRightDrive, robot.backLeftDrive, robot.backRightDrive);

        if(utilFuncs.outOfDeadband(liftPower, 0.1))
        {
            if (liftPower > 0) {
                m_lift.raiseLift(liftPower);
            } else {
                m_lift.lowerLift(liftPower);
            }
        } else {
            m_lift.kill();
        }

        if(gamepad2.a) {
            m_intake.wheelIn();
        } else {
            m_intake.wheelKill();
        }

        telemetry.addData("Forward Value: ",  forward);
        telemetry.addData("Strafe Value:  ", strafe);
        telemetry.addData("Rotate Value:  ",  rotate);

        telemetry.addData("Gamepad1 - LeftStickX", gamepad1.left_stick_x);
        telemetry.addData("Gamepad1 - LeftStickY", gamepad1.left_stick_y);
        telemetry.addData("Gamepad1 - RightStickX", gamepad1.right_stick_x);
        telemetry.addData("Gamepad1 - RightStickY", gamepad1.right_stick_y);
        telemetry.addLine();
        telemetry.addData("Gamepad2 - LeftStickX", gamepad2.left_stick_x);
        telemetry.addData("Gamepad2 - LeftStickY", gamepad2.left_stick_y);
        telemetry.addData("Gamepad2 - RightStickX", gamepad2.right_stick_x);
        telemetry.addData("Gamepad2 - RightStickY", gamepad2.right_stick_y);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}

