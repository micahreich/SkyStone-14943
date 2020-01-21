package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public HardwareMap hwMap;
    public LinearOpMode opMode;

    public SubsystemDriveTrainMecanum subsystemDriveTrainMecanum;
    public SubsystemIntake subsystemIntake;
    public SubsystemFoundationGrabber subsystemFoundationGrabber;
    public SubsystemLift subsystemLift;
    public SubsystemIMU subsystemIMU;

    public Robot(HardwareMap hwMap, LinearOpMode opMode) {
        hwMap = this.hwMap;
        opMode = this.opMode;

        subsystemDriveTrainMecanum= new SubsystemDriveTrainMecanum(hwMap, opMode);
        subsystemIntake = new SubsystemIntake(hwMap, opMode);
        subsystemFoundationGrabber = new SubsystemFoundationGrabber(hwMap, opMode);
        subsystemLift = new SubsystemLift(hwMap, opMode);
        subsystemIMU = new SubsystemIMU(hwMap, opMode);
    }

    public void initRobot() {
        subsystemDriveTrainMecanum.initHardware("leftFront", "leftBack", "rightFront", "rightBack");
        subsystemIntake.initHardware("rightIntake", "leftIntake");
        subsystemFoundationGrabber.initHardware("rightGrabber", "leftGrabber");
        subsystemLift.initHardware("rightLift", "leftLift");
        subsystemIMU.initHardware("imu");

        opMode.telemetry.addLine("ROBOT HARDWARE: INITIALIZED");
        opMode.telemetry.update();
    }

}
