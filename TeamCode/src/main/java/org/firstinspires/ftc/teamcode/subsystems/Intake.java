package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake {
    public static final double THRESHOLD = 11;

    private DcMotor m_left, m_right;
    private DistanceSensor s_sensorRange;

    /**
     * Make sure all motors are inverted properly before passing into the constructor
     */
    public Intake(DcMotor left, DcMotor right, DistanceSensor sensorRange) {
        s_sensorRange = sensorRange;
        m_left = left;
        m_right = right;
    }

    public void runIntake(double power) {
        m_left.setPower(power);
        m_right.setPower(power);
    }

    public boolean stoneInBay() {
        return s_sensorRange.getDistance(DistanceUnit.CM) < THRESHOLD;
    }

    public void disable() {
        m_left.close();
        m_right.close();
    }
}
