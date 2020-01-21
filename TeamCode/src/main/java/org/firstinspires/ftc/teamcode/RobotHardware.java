package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotHardware {

    // public opMode members
    public DcMotor leftFront, rightFront, leftBack, rightBack;

    // local opMode members
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    // empty class constructor
    public RobotHardware() {

    }

    // initialize standard hardware interfaces
    public void initi(HardwareMap ahwMap) {
        // save reference to Hardware map
        hwMap = ahwMap;

        // define and initialize motors for drivetrain
        leftFront  = hwMap.get(DcMotor.class, "leftFront");
        rightFront = hwMap.get(DcMotor.class, "rightFront");
        leftBack   = hwMap.get(DcMotor.class, "leftBack");
        rightBack  = hwMap.get(DcMotor.class, "rightBack");
    }
}
