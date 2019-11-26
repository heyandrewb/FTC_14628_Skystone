package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.armservos.Elbow;
import org.firstinspires.ftc.teamcode.armservos.Extension;
import org.firstinspires.ftc.teamcode.armservos.Gripper;


/**
 * Example Autonomous Opmode
 * <p>
 * Uses Line-following two drive around the tape at the perimeter of the lander.
 * <p>
 * Requires mechanum bot configuration.
 * <p>
 * Start with bot in center of lander, facing top of screen.
 * <p>
 * Disabling for now; it was designed to work with Rover Ruckus field
 */
//@Disabled
@Autonomous(name = "Skystone146826 Auto", group = "Mechanum")
public class SkystoneAutonomous extends LinearOpMode {
    SkystoneHardwareMap robot = new SkystoneHardwareMap();
    //GyroSensor gyro;
    BNO055IMU imu;
    ColorSensor colorSensor;

    DistanceSensor leftDistanceSensor;
    DistanceSensor rightDistanceSensor;

    Elbow elbow = new Elbow(robot.elbowServo);
    Extension extension = new Extension(robot.extensionServo, robot.extensionLimitOut, robot.extensionLimitIn);
    Gripper gripper = new Gripper(robot.gripperServo, robot.extensionLimitOut, robot.extensionLimitIn);

    Lift lift = new Lift(robot.liftMotor, robot.liftDownLimit, robot.liftUpLimit);
    Intake intake = new Intake(robot.leftIntakeServo, robot.rightIntakeServo, robot.leftIntakeWheel, robot.rightIntakeWheel);

    static final double DRIVE_SPEED = .5;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.5;     // Nominal half speed for better accuracy.

    //    static final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: TETRIX Motor Encoder
    static final double COUNTS_PER_MOTOR_REV = 1024.0;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double HEADING_THRESHOLD = 3;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable

    double blockDistance = 0.0;

    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {

        robot.backLeftDrive = hardwareMap.dcMotor.get("back_left_motor");
        robot.frontLeftDrive = hardwareMap.dcMotor.get("front_left_motor");
        robot.frontRightDrive = hardwareMap.dcMotor.get("front_right_motor");
        robot.backRightDrive = hardwareMap.dcMotor.get("back_right_motor");
        robot.backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        robot.frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(new BNO055IMU.Parameters());

        colorSensor = hardwareMap.colorSensor.get("color_sensor");

        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "left_distance");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "right_distance");

        Orientation orientation;

        ElapsedTime waitTime = new ElapsedTime();
        while (!opModeIsActive() && !isStopRequested()) {
            String color = "blue";
            telemetry.addData("Seconds since init", "%d. Press start when ready.", (int) waitTime.seconds());
            telemetry.update();
        }

        //BuildPlateAuto("red", true, 5);
        skystoneBlueAuto("right");
//        skystoneRedAuto("left");

        killDrive();
    }

    public void skystoneBlueAuto(String blockSelection) {

        if (blockSelection.equals("left")) {
            //Strafe Left
            while (opModeIsActive() && rightDistanceSensor.getDistance(DistanceUnit.INCH) < 35.5) {
                robot.frontLeftDrive.setPower(-DRIVE_SPEED);
                robot.frontRightDrive.setPower(DRIVE_SPEED);
                robot.backLeftDrive.setPower(DRIVE_SPEED);
                robot.backRightDrive.setPower(-DRIVE_SPEED);
            }
            killDrive();
            sleep(500);
            gyroDrive(DRIVE_SPEED, 32.5, 0);
            sleep(500);
        } else if (blockSelection.equals("center")) {
            //Strafe Left
            while (opModeIsActive() && rightDistanceSensor.getDistance(DistanceUnit.INCH) < 27.5) {
                robot.frontLeftDrive.setPower(-DRIVE_SPEED);
                robot.frontRightDrive.setPower(DRIVE_SPEED);
                robot.backLeftDrive.setPower(DRIVE_SPEED);
                robot.backRightDrive.setPower(-DRIVE_SPEED);
            }
            killDrive();
            sleep(500);
            gyroDrive(DRIVE_SPEED, 32.5, 0);
            sleep(500);
        } else if (blockSelection.equals("right")) {
            //Strafe Left
            while (opModeIsActive() && rightDistanceSensor.getDistance(DistanceUnit.INCH) < 25) {
                robot.frontLeftDrive.setPower(-DRIVE_SPEED);
                robot.frontRightDrive.setPower(DRIVE_SPEED);
                robot.backLeftDrive.setPower(DRIVE_SPEED);
                robot.backRightDrive.setPower(-DRIVE_SPEED);
            }
            killDrive();
            sleep(500);
            gyroDrive(DRIVE_SPEED, 30.0, 0);
            sleep(500);
            while (opModeIsActive() && rightDistanceSensor.getDistance(DistanceUnit.INCH) > 19.5) {
                robot.frontLeftDrive.setPower(DRIVE_SPEED);
                robot.frontRightDrive.setPower(-DRIVE_SPEED);
                robot.backLeftDrive.setPower(-DRIVE_SPEED);
                robot.backRightDrive.setPower(DRIVE_SPEED);
            }
            killDrive();
            sleep(500);
            gyroDrive(DRIVE_SPEED, 2.5, 0);
            killDrive();
            sleep(500);
        } else {
            killDrive();
        }
    }

    public void BuildPlateAuto(String color, boolean shouldPullForward, double pullForwardDelay) {
        DistanceSensor distanceSensor = color.equals("blue") ? leftDistanceSensor : rightDistanceSensor;

        while (opModeIsActive() && distanceSensor.getDistance(DistanceUnit.INCH) > 19) {
            if (color.equals("blue")) {
                strafeLeft(DRIVE_SPEED);
            } else {
                strafeRight(DRIVE_SPEED);
            }
        }

        gyroDrive(DRIVE_SPEED, 32.5, 0.0);
        lift.raiseLift(1);
        intake.lower();
        sleep(500);
        if(color.equals("blue"))
        {
            arcDrive(11,90,DRIVE_SPEED,true,false);
        }
        else
        {
            arcDrive(11,90,DRIVE_SPEED,false,true);
        }
        sleep(500);
//        gyroDrive(DRIVE_SPEED, -32.5, 0.0);

        sleep(500);
        if (shouldPullForward) {
            ElapsedTime delayTime = new ElapsedTime();
            for (double x = 0.0; x < pullForwardDelay * 1000; x += 100) {
                sleep(100);
                telemetry.addData("Delay", round((pullForwardDelay) - delayTime.seconds(), 1));
                telemetry.update();
            }
            if(color.equals("blue"))
            {
                while (opModeIsActive() && leftDistanceSensor.getDistance(DistanceUnit.INCH) < 26)
                    strafeRight(DRIVE_SPEED);
            }
            else
            {
                while (opModeIsActive() && rightDistanceSensor.getDistance(DistanceUnit.INCH) < 26)
                    strafeLeft(DRIVE_SPEED);
            }
        }
        double tapeDetection = color.equals("blue") ? colorSensor.red() : colorSensor.blue();

        while (opModeIsActive() && tapeDetection > 100) {
            tapeDetection = color.equals("blue") ? colorSensor.red() : colorSensor.blue();
            driveBackward(DRIVE_SPEED);
        }
        killDrive();
    }

    public void skystoneRedAuto(String blockSelection) {
        if (blockSelection.equals("right")) {
            //Strafe Left
            while (opModeIsActive() && leftDistanceSensor.getDistance(DistanceUnit.INCH) < 35.5) {
                robot.frontLeftDrive.setPower(DRIVE_SPEED);
                robot.frontRightDrive.setPower(-DRIVE_SPEED);
                robot.backLeftDrive.setPower(-DRIVE_SPEED);
                robot.backRightDrive.setPower(DRIVE_SPEED);
            }
            killDrive();
            sleep(500);
            gyroDrive(DRIVE_SPEED, 32.5, 0);
            sleep(500);
        } else if (blockSelection.equals("center")) {
            //Strafe Left
            while (opModeIsActive() && leftDistanceSensor.getDistance(DistanceUnit.INCH) < 27.5) {
                robot.frontLeftDrive.setPower(DRIVE_SPEED);
                robot.frontRightDrive.setPower(-DRIVE_SPEED);
                robot.backLeftDrive.setPower(-DRIVE_SPEED);
                robot.backRightDrive.setPower(DRIVE_SPEED);
            }
            killDrive();
            sleep(500);
            gyroDrive(DRIVE_SPEED, 32.5, 0);
            sleep(500);
        } else if (blockSelection.equals("left")) {
            //Strafe Left
            while (opModeIsActive() && leftDistanceSensor.getDistance(DistanceUnit.INCH) < 25) {
                robot.frontLeftDrive.setPower(DRIVE_SPEED);
                robot.frontRightDrive.setPower(-DRIVE_SPEED);
                robot.backLeftDrive.setPower(-DRIVE_SPEED);
                robot.backRightDrive.setPower(DRIVE_SPEED);
            }
            killDrive();
            sleep(500);
            gyroDrive(DRIVE_SPEED, 30.0, 0);
            sleep(500);
            while (opModeIsActive() && leftDistanceSensor.getDistance(DistanceUnit.INCH) > 19.5) {
                robot.frontLeftDrive.setPower(-DRIVE_SPEED);
                robot.frontRightDrive.setPower(DRIVE_SPEED);
                robot.backLeftDrive.setPower(DRIVE_SPEED);
                robot.backRightDrive.setPower(-DRIVE_SPEED);
            }
            killDrive();
            sleep(500);
            gyroDrive(DRIVE_SPEED, 2.5, 0);
            killDrive();
            sleep(500);
        } else {
            killDrive();
        }
    }

    public void gyroDrive(double speed, double distance, double angle) {

        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);
            newLeftTarget = (robot.frontLeftDrive.getCurrentPosition() + robot.backLeftDrive.getCurrentPosition()) / 2 + moveCounts;
            newRightTarget = (robot.frontRightDrive.getCurrentPosition() + robot.backRightDrive.getCurrentPosition()) / 2 + moveCounts;


            // Set Target and Turn On RUN_TO_POSITION
            robot.frontLeftDrive.setTargetPosition(newLeftTarget);
            robot.backLeftDrive.setTargetPosition(newLeftTarget);
            robot.frontRightDrive.setTargetPosition(newRightTarget);
            robot.backRightDrive.setTargetPosition(newRightTarget);

            robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Math.abs(speed);
            robot.frontLeftDrive.setPower(speed);
            robot.frontRightDrive.setPower(speed);
            robot.backLeftDrive.setPower(speed);
            robot.backRightDrive.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.frontLeftDrive.isBusy() && robot.frontRightDrive.isBusy() && robot.backLeftDrive.isBusy() && robot.backRightDrive.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.frontLeftDrive.setPower(leftSpeed);
                robot.backLeftDrive.setPower(leftSpeed);
                robot.frontRightDrive.setPower(rightSpeed);
                robot.backRightDrive.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Actual", "%7d:%7d", robot.frontLeftDrive.getCurrentPosition(),
                        robot.frontRightDrive.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.frontLeftDrive.setPower(0);
            robot.frontRightDrive.setPower(0);
            robot.backLeftDrive.setPower(0);
            robot.backRightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void gyroTurn(double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

//    public void gyroHold( double speed, double angle, double holdTime)
//    {
//
//        ElapsedTime holdTimer = new ElapsedTime();
//
//        // keep looping while we have time remaining.
//        holdTimer.reset();
//        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
//            // Update telemetry & Allow time for other processes to run.
//            onHeading(speed, angle, P_TURN_COEFF);
//            telemetry.update();
//        }
//
//        // Stop all motion;
//        robot.frontLeftDrive.setPower(0);
//        robot.frontRightDrive.setPower(0);
//        robot.backLeftDrive.setPower(0);
//        robot.backRightDrive.setPower(0);
//    }

    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.frontLeftDrive.setPower(leftSpeed);
        robot.frontRightDrive.setPower(rightSpeed);
        robot.backLeftDrive.setPower(leftSpeed);
        robot.backRightDrive.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - imu.getAngularOrientation().firstAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return clip(error * PCoeff, -1, 1);
    }

    public double clip(double input, double min, double max) {
        if (input > max) {
            input = max;
        } else if (input < min) {
            input = min;
        } else {
            input = input;
        }
        return input;
    }

    public void killDrive() {
        robot.frontLeftDrive.setPower(0);
        robot.backLeftDrive.setPower(0);
        robot.frontRightDrive.setPower(0);
        robot.backRightDrive.setPower(0);
    }

    public void driveForward(double speed)
    {
        robot.frontLeftDrive.setPower(Math.abs(speed));
        robot.frontRightDrive.setPower(Math.abs(speed));
        robot.backLeftDrive.setPower(Math.abs(speed));
        robot.backRightDrive.setPower(Math.abs(speed));
    }

    public void driveBackward(double speed)
    {
        robot.frontLeftDrive.setPower(Math.abs(speed) * -1);
        robot.frontRightDrive.setPower(Math.abs(speed) * -1);
        robot.backLeftDrive.setPower(Math.abs(speed) * -1);
        robot.backRightDrive.setPower(Math.abs(speed) * -1);
    }

    public void strafeLeft(double speed) {
        robot.frontLeftDrive.setPower(-speed);
        robot.frontRightDrive.setPower(speed);
        robot.backLeftDrive.setPower(speed);
        robot.backRightDrive.setPower(-speed);
    }

    public void strafeRight(double speed) {
        robot.frontLeftDrive.setPower(speed);
        robot.frontRightDrive.setPower(-speed);
        robot.backLeftDrive.setPower(-speed);
        robot.backRightDrive.setPower(speed);
    }

    public double round(double input, int decimalPlaces) {
        return ((int) (input * Math.pow(10, decimalPlaces)) / Math.pow(10, decimalPlaces));
    }

    public void arcDrive(double radius, double degrees, double speed, boolean clockwise, boolean pointIsLeft) {
        double trackWidth = 15.4160336;
        degrees *= 2;
        double circumferenceInner = (radius*.81 - (trackWidth * .615)) * 2.0 * Math.PI; //calculate circumference of circle inner wheels follow
        double circumferenceOuter = (radius*.81 + (trackWidth * .45)) * 2.0 * Math.PI; //calculate circumference of circle outer wheels follow

        double distanceInner = circumferenceInner * (degrees / 360.0); //calculate distance inner wheels will travel based on ratio of degrees
        double distanceOuter = circumferenceOuter * (degrees / 360.0); //calculate distance outer wheels will travel based on ratio of degrees

        double powerInner = clip(speed * (distanceInner / distanceOuter), -1, 1); //Set power to 1 * speed
        double powerOuter = clip(speed * (distanceOuter / distanceOuter), -1, 1); //Find ratio that makes both outer and inner wheels end at the same time.

        if (pointIsLeft) {
            if (!clockwise) {
                powerInner *= -1;
                powerOuter *= -1;

                distanceInner *= -1;
                distanceOuter *= -1;
            }
            encoderArcDrive(distanceInner, powerInner, distanceOuter, powerOuter, 5);
        } else {
            if (clockwise) {
                powerInner *= -1;
                powerOuter *= -1;

                distanceInner *= -1;
                distanceOuter *= -1;
            }
            encoderArcDrive(distanceOuter, powerOuter, distanceInner, powerInner, 5);
        }
    }

    public void encoderArcDrive(double leftDistance, double leftSpeed, double rightDistance, double rightSpeed, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            newLeftTarget = (robot.frontLeftDrive.getCurrentPosition() + robot.backLeftDrive.getCurrentPosition())/2 + (int)(leftDistance * COUNTS_PER_INCH);
            newRightTarget = (robot.frontRightDrive.getCurrentPosition() + robot.backRightDrive.getCurrentPosition())/2 + (int)(rightDistance * COUNTS_PER_INCH);
            robot.frontLeftDrive.setTargetPosition(newLeftTarget);
            robot.backLeftDrive.setTargetPosition(newLeftTarget);
            robot.frontRightDrive.setTargetPosition(newRightTarget);
            robot.backRightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.frontLeftDrive.setPower(Math.abs(leftSpeed));
            robot.backLeftDrive.setPower(Math.abs(leftSpeed));
            robot.frontRightDrive.setPower(Math.abs(rightSpeed));
            robot.backRightDrive.setPower(Math.abs(rightSpeed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.frontLeftDrive.isBusy() || robot.frontRightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.frontLeftDrive.getCurrentPosition(),
                        robot.frontRightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            killDrive();

            // Turn off RUN_TO_POSITION
            robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }}
