package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.PIDController;

public class Lift {

    public static final double STONE_TICKS = 300; // tune this value @micah
    public static final double THRESHOLD = 10;    // tune this value @micah

    private int stackHeight;
    private DcMotor m_left, m_right;
    private PIDController m_controller;

    /**
     * Make sure all motors are inverted properly before passing into the constructor
     */
    public Lift(DcMotor left, DcMotor right, PIDController controller) {
        m_left = left;
        m_right = right;

        m_controller = controller;
    }

    public void driveLift(double power) {
        m_left.setPower(power);
        m_right.setPower(power);
    }

    public void disable() {
        m_left.close();
        m_right.close();
    }

    public int getStackHeight() {
        return stackHeight;
    }

    public void addStoneToStack() {
        stackHeight++;
    }

    public double getLiftPosition() {
        return (m_left.getDirection() == DcMotor.Direction.FORWARD) ?
                m_left.getCurrentPosition() : m_right.getCurrentPosition();
    }

    public void moveLiftUpToNextStoneLevel() {
        double setVal = STONE_TICKS * stackHeight;
        m_controller.setPoint(setVal);

        while (Math.abs(setVal - getLiftPosition()) < THRESHOLD) {
            driveLift(m_controller.update(getLiftPosition()));
        }

        addStoneToStack();
    }

    public void reset() {
        m_controller.setPoint(0);

        while (Math.abs(0 - getLiftPosition()) < THRESHOLD) {
            driveLift(m_controller.update(getLiftPosition()));
        }
    }

}
