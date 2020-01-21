package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class TeleOpMain extends LinearOpMode {
    Robot robot = new Robot(hardwareMap, this);
    GamepadHandler controls = new GamepadHandler(robot, this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initRobot();

        waitForStart();
        boolean lastState = false;
        int liftHeight = 0;

        while(opModeIsActive() && !isStopRequested()) {
            robot.subsystemDriveTrainMecanum.arcadeDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            controls.runControls(liftHeight);
        }
    }
}
