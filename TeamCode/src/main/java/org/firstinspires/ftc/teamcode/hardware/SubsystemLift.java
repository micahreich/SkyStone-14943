package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SubsystemLift {
    public HardwareMap hwMap;
    public LinearOpMode opMode;
    public Telemetry telemetry;

    public DcMotor[] liftMotors = new DcMotor[2];

    public SubsystemLift(HardwareMap hwMap, Telemetry telemetry, LinearOpMode opMode) {
        hwMap = this.hwMap;
        opMode = this.opMode;
        telemetry = this.telemetry;

        telemetry.addLine("LIFT: INSTANTIATED");
        telemetry.update();
    }

    public void initHardware(String rightMotor, String leftMotor) {
        liftMotors[0] = hwMap.get(DcMotor.class, rightMotor);
        liftMotors[1] = hwMap.get(DcMotor.class, leftMotor);

        liftMotors[0].setDirection(DcMotor.Direction.FORWARD);
        liftMotors[2].setDirection(DcMotor.Direction.REVERSE);


        for(DcMotor dcMotor : liftMotors) {
            dcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        telemetry.addLine("LIFT: INITIALIZED");
        telemetry.update();
    }

    public void raiseLift() {
        double liftHeight = 0;
    }

    public void lowerLift() {
        double liftHeight = 0;
    }

}
