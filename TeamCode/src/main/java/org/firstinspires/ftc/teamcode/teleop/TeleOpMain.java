package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class TeleOpMain extends LinearOpMode {
    RobotHardware robot = new RobotHardware(hardwareMap, telemetry, this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initRobot();

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            robot.subsystemDriveTrainMecanum.arcadeDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        }
    }
}
