package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SubsystemFoundationGrabber {
    public HardwareMap hwMap;
    public Telemetry telemetry;
    public LinearOpMode opMode;

    public final double GRABBING = 1.0;
    public final double IDLE = 0.0;

    public Servo[] foundationGrabbers = new Servo[2];

    public SubsystemFoundationGrabber(HardwareMap hwMap, Telemetry telemetry, LinearOpMode opMode) {
        hwMap = this.hwMap;
        telemetry = this.telemetry;
        opMode = this.opMode;

        telemetry.addLine("FOUNDATION GRABBER: INSTANTIATED");
        telemetry.update();
    }

    public void initHardware(String rightGrabber, String leftGrabber) {
        foundationGrabbers[0] = hwMap.get(Servo.class, rightGrabber);
        foundationGrabbers[1] = hwMap.get(Servo.class, leftGrabber);

        telemetry.addLine("FOUNDATION GRABBER: INITIALIZED");
        telemetry.update();
    }

    public void setIdlePosition() {
        foundationGrabbers[0].setPosition(1 - IDLE);
        foundationGrabbers[1].setPosition(IDLE);
    }

    public void setGrabbingPosition() {
        foundationGrabbers[0].setPosition(1 - GRABBING);
        foundationGrabbers[1].setPosition(GRABBING);
    }
}
