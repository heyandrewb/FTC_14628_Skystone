package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Sensors {
    ColorSensor m_colorSensor;
    DistanceSensor m_leftDist;
    DistanceSensor m_rightDist;

    public Sensors(ColorSensor colorSensor, DistanceSensor leftDist, DistanceSensor rightDist) {
        m_colorSensor = colorSensor;
        m_leftDist = leftDist;
        m_rightDist = rightDist;
    }

    public int getColor() {
        return m_colorSensor.argb();
    }

    public double getLeftDist() {
        return m_leftDist.getDistance(DistanceUnit.INCH);
    }

    public double getRightDist() {
        return m_rightDist.getDistance(DistanceUnit.INCH);
    }
}
