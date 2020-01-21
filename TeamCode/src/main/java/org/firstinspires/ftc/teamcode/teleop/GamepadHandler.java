package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.hardware.Robot;

public class GamepadHandler {
    public Robot robot;
    public LinearOpMode opMode;
    public Gamepad gamepad1;
    public Gamepad gamepad2;

    public final double INTAKE_SPEED_IN = 1.0;
    public final double INTAKE_SPEED_OUT = -1.0;

    public GamepadHandler(Robot robot, LinearOpMode opMode) {
        robot = this.robot;
        opMode = this.opMode;

        gamepad1 = opMode.gamepad1;
        gamepad2 = opMode.gamepad2;

        opMode.telemetry.addLine("GAMEPAD HANDLER DEFINED");
        opMode.telemetry.update();
    }

    public void runControls() {
        if(gamepad1.right_bumper) {
            robot.subsystemIntake.runIntake(INTAKE_SPEED_IN);
        } else if(gamepad1.left_bumper) {
            robot.subsystemIntake.runIntake(INTAKE_SPEED_OUT);
        } else if(gamepad1.y) {
            robot.subsystemFoundationGrabber.setGrabbingPosition();
        } else if(gamepad1.a) {
            robot.subsystemFoundationGrabber.setIdlePosition();
        }

        if(gamepad2.dpad_up) {
            robot.subsystemLift.raiseLift();
        } else if(gamepad2.dpad_down) {
            robot.subsystemLift.lowerLift();
        }
    }
}
