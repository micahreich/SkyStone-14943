package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SubsystemFoundationGrabber {
    public HardwareMap hwMap;
    public LinearOpMode opMode;

    public final double GRABBING = 1.0;
    public final double IDLE = 0.0;

    public Servo[] foundationGrabbers = new Servo[2];

    public SubsystemFoundationGrabber(HardwareMap hwMap, LinearOpMode opMode) {
        hwMap = this.hwMap;
        opMode = this.opMode;

        opMode.telemetry.addLine("FOUNDATION GRABBER: INSTANTIATED");
        opMode.telemetry.update();
    }

    public void initHardware(String rightGrabber, String leftGrabber) {
        foundationGrabbers[0] = hwMap.get(Servo.class, rightGrabber);
        foundationGrabbers[1] = hwMap.get(Servo.class, leftGrabber);

        opMode.telemetry.addLine("FOUNDATION GRABBER: INITIALIZED");
        opMode.telemetry.update();
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
