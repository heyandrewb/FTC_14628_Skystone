package org.firstinspires.ftc.teamcode;

public class utilFuncs {
    public static boolean outOfDeadband(double input, double deadband) {
        if (input > deadband || input < -deadband) {
            return true;
        } else {
            return false;
        }
    }
}
