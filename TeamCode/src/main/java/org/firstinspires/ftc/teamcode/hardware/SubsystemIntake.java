package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SubsystemIntake {
    public HardwareMap hwMap;
    public LinearOpMode opMode;

    public DcMotor[] intakeMotors = new DcMotor[2];

    public SubsystemIntake(HardwareMap hwMap, LinearOpMode opMode) {
        hwMap = this.hwMap;
        opMode = this.opMode;

        opMode.telemetry.addLine("INTAKE: INSTANTIATED");
        opMode.telemetry.update();
    }

    public void initHardware(String rightModule, String leftModule) {
        intakeMotors[0] = hwMap.get(DcMotor.class, rightModule);
        intakeMotors[1] = hwMap.get(DcMotor.class, leftModule);

        intakeMotors[0].setDirection(DcMotor.Direction.FORWARD);
        intakeMotors[1].setDirection(DcMotor.Direction.REVERSE);

        for(DcMotor dcMotor : intakeMotors) {
            dcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        opMode.telemetry.addLine("INTAKE: INITIALIZED");
        opMode.telemetry.update();
    }

    public void runIntake(double power) {
        for(DcMotor dcMotor : intakeMotors) {
            dcMotor.setPower(power);
        }
    }
}
