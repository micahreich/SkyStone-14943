package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotHardware {
    public HardwareMap hwMap;
    public Telemetry telemetry;
    public LinearOpMode opMode;

    public SubsystemDriveTrainMecanum subsystemDriveTrainMecanum;
    public SubsystemIntake subsystemIntake;
    public SubsystemFoundationGrabber subsystemFoundationGrabber;
    public SubsystemLift subsystemLift;
    public SubsystemIMU subsystemIMU;

    public RobotHardware(HardwareMap hwMap, Telemetry telemetry, LinearOpMode opMode) {
        hwMap = this.hwMap;
        telemetry = this.telemetry;
        opMode = this.opMode;

        subsystemDriveTrainMecanum= new SubsystemDriveTrainMecanum(hwMap, telemetry, opMode);
        subsystemIntake = new SubsystemIntake(hwMap, telemetry, opMode);
        subsystemFoundationGrabber = new SubsystemFoundationGrabber(hwMap, telemetry, opMode);
        subsystemLift = new SubsystemLift(hwMap, telemetry, opMode);
        subsystemIMU = new SubsystemIMU(hwMap, telemetry, opMode);
    }

    public void initRobot() {
        subsystemDriveTrainMecanum.initHardware("leftFront", "leftBack", "rightFront", "rightBack");
        subsystemIntake.initHardware("rightIntake", "leftIntake");
        subsystemFoundationGrabber.initHardware("rightGrabber", "leftGrabber");
        subsystemLift.initHardware("rightLift", "leftLift");
        subsystemIMU.initHardware("imu");

        telemetry.addLine("ROBOT HARDWARE: INITIALIZED");
        telemetry.update();
    }

}
