package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SkystoneHardwareMap {

    /**
     * This is NOT an opmode.
     * <p>
     * This hardware class assumes the following device names have been configured on the robot:
     * Note:  All names are lower camelCase
     * <p>
     * Motor channel:  Front Left Drive motor:   "frontLeftDrive"
     * Motor channel:  Front Right Drive motor:  "frontRightDrive"
     * Motor channel:  Back Left Drive motor:    "backLeftDrive"
     * Motor channel:  Back Right Drive motor:   "backRightDrive"
     * Servo channel:  Servo for Left Intake:    "leftIntakeServo"
     * Servo channel:  Servo for Right Intake:   "rightIntakeServo"
     */

    /* Public OpMode members. */
    public DcMotor frontLeftDrive = null;
    public DcMotor frontRightDrive = null;
    public DcMotor backLeftDrive = null;
    public DcMotor backRightDrive = null;

    // lift
    public DcMotor liftMotor = null;
    public TouchSensor liftUpLimit = null;
    public TouchSensor liftDownLimit = null;

    // intake
    public Servo leftIntakeServo = null;
    public Servo rightIntakeServo = null;
    public DcMotor leftIntakeWheel = null;
    public DcMotor rightIntakeWheel = null;

    // arm
    public Servo elbowServo = null;
    public CRServo extensionServo = null;
    public CRServo gripperServo = null;
    public TouchSensor extensionLimitIn = null;
    public TouchSensor extensionLimitOut = null;
    public TouchSensor gripperLimitIn = null;
    public TouchSensor gripperLimitOut = null;

    // sensors
    public ColorSensor colorSensor = null;
    public DistanceSensor distLeft = null;
    public DistanceSensor distRight = null;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeftDrive = hwMap.get(DcMotor.class, "FrontLeft");
        frontRightDrive = hwMap.get(DcMotor.class, "FrontRight");
        backLeftDrive = hwMap.get(DcMotor.class, "BackLeft");
        backRightDrive = hwMap.get(DcMotor.class, "BackRight");

        liftMotor = hwMap.get(DcMotor.class, "Lift");

        leftIntakeWheel = hwMap.get(DcMotor.class, "LeftIntakeWheel");
        rightIntakeWheel = hwMap.get(DcMotor.class, "RightIntakeWheel");

        // Set motor directions
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        liftMotor.setDirection(DcMotor.Direction.FORWARD);

        leftIntakeWheel.setDirection(DcMotor.Direction.FORWARD);
        rightIntakeWheel.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);

        liftMotor.setPower(0);

        leftIntakeWheel.setPower(0);
        rightIntakeWheel.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftIntakeWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntakeWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
//        leftIntakeServo = hwMap.get(Servo.class, "LeftIntakeServo");
//        rightIntakeServo = hwMap.get(Servo.class, "RightIntakeServo");
//
//        elbowServo = hwMap.get(Servo.class, "ElbowServo");
//        extensionServo = hwMap.get(CRServo.class, "ExtensionServo");
//        gripperServo = hwMap.get(CRServo.class, "GripperServo");
//
//        // Define and initialize limit switches.
//        liftUpLimit = hwMap.get(TouchSensor.class, "LiftUpLimit");
//        liftDownLimit = hwMap.get(TouchSensor.class, "LiftDownLimit");
//        extensionLimitIn = hwMap.get(TouchSensor.class, "ExtensionLimitIn");
//        extensionLimitOut = hwMap.get(TouchSensor.class, "ExtencionLimitOut");
//        gripperLimitIn = hwMap.get(TouchSensor.class, "GripperLimitIn");
//        gripperLimitOut = hwMap.get(TouchSensor.class, "GripperLimitOut");
//
//        // Define and initialize sensors
//        colorSensor = hwMap.get(ColorSensor.class, "ColorSensor");
//        distLeft = hwMap.get(DistanceSensor.class, "LeftDistSensor");
//        distRight = hwMap.get(DistanceSensor.class, "RightDistSensor");
    }
}


