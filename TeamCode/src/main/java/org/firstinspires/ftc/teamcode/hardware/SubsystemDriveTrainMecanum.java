package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SubsystemDriveTrainMecanum {
    public SubsystemIMU subsystemIMU;

    public final double TICKS_PER_REVOLUTION = 537.6;
    public final double WHEEL_DIAMETER_IN = 100 / 25.4;
    public final double INITIAL_DRIVE_VELOCITY = 0.2;

    public final double kSpeed = 1.0;
    public final double kDistance = 1.0;

    public final double kAngle = 1.0;
    public final double kOffset = 1.0;

    public HardwareMap hwMap;
    public LinearOpMode opMode;

    public DcMotor[] driveMotors = new DcMotor[4];

    public SubsystemDriveTrainMecanum(HardwareMap hwMap, OpMode opMode) {
        /***
         * SubsystemDriveTrainMecanum is the class constructor
         * @param hwMap the hardwareMap object expected from classes which instantiate this class
         * @param opMode the opMode object expected from classes which instantiate this class
         */

        hwMap = this.hwMap;
        opMode = this.opMode;

        opMode.telemetry.addLine("MECANUM DRIVETRAIN: INSTANTIATED");
        opMode.telemetry.update();
    }

    public void initHardware(String leftFront, String leftBack, String rightFront, String rightBack) {
        /***
         * initHardware initializes all drivetrain hardware from the hardwareMap
         * @param leftFront hardwareMap motor reference name
         * @param leftBack hardwareMap motor reference name
         * @param rightFront hardwareMap motor reference name
         * @param rightBack hardwareMap motor reference name
         */

        driveMotors[0] = hwMap.get(DcMotor.class, leftFront);
        driveMotors[1] = hwMap.get(DcMotor.class, leftBack);
        driveMotors[2] = hwMap.get(DcMotor.class, rightFront);
        driveMotors[3] = hwMap.get(DcMotor.class, rightBack);

        driveMotors[0].setDirection(DcMotor.Direction.REVERSE);
        driveMotors[2].setDirection(DcMotor.Direction.REVERSE);


        for(DcMotor dcMotor : driveMotors) {
            dcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        opMode.telemetry.addLine("MECANUM DRIVETRAIN: INITIALIZED");
        opMode.telemetry.update();
    }

    public void arcadeDrive(float xDriveVector, float yDriveVector, float turnVector) {
        /***
         * arcadeDrive handles the arcade drive capabilities for TeleOp driving
         * @param xDriveVector vector representing x component of driving forwards/backwards/strafing
         * @param yDriveVector vector representing y component of driving forwards/backwards/strafing
         * @param turnVector vector representing x component of turning
         */

        double speed = Math.hypot(xDriveVector, yDriveVector);
        double angle = Math.atan2(yDriveVector, xDriveVector) - Math.PI/4;
        double turn = turnVector;

        final double lfPower = speed * Math.sin(angle) + turn;
        final double rfPower = speed * Math.cos(angle) - turn;
        final double lbPower = speed * Math.cos(angle) + turn;
        final double rbPower = speed * Math.sin(angle) - turn;

        setDrivePowers(lfPower, rfPower, lbPower, rbPower);

    }

    public void translateY(double targetPosition, double maxVelocity, double accelerationPeriod) {
        /***
         * translateY handles driving forwards and backwards for autonomous driving
         * @param targetPosition target encoder ticks
         * @param maxVelocity maximum motor power the drivetrain should reach
         * @param accelerationPeriod the period of acceleration and deceleration for driving
         */

        targetPosition = targetPosition * kDistance;

        for(DcMotor dcMotor : driveMotors) {
            dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        if(targetPosition > 0) {     // forwards
            while (!opMode.opModeIsActive() && !opMode.isStopRequested() && (driveMotors[0].getCurrentPosition() < inchesToTicks(targetPosition) ||
                    driveMotors[1].getCurrentPosition() < inchesToTicks(targetPosition) ||
                    driveMotors[2].getCurrentPosition() < inchesToTicks(targetPosition) ||
                    driveMotors[3].getCurrentPosition() < inchesToTicks(targetPosition))) {

                double power = trapezoidMotionProfile(
                        driveMotors[0].getCurrentPosition(),
                        targetPosition,
                        maxVelocity, INITIAL_DRIVE_VELOCITY,
                        (accelerationPeriod * targetPosition)
                );

                setDrivePowers(power, power, power, power);

                opMode.telemetry.addData("MECANUM DRIVETRAIN: RUNNING TO POSITION %",
                        (1 - (((inchesToTicks(targetPosition) - driveMotors[0].getCurrentPosition()) / inchesToTicks(targetPosition)) * 100)));
                opMode.telemetry.update();
            }
        } else if(targetPosition < 0) {     // backwards
            while (!opMode.opModeIsActive() && !opMode.isStopRequested() && (driveMotors[0].getCurrentPosition() > inchesToTicks(targetPosition) ||
                    driveMotors[1].getCurrentPosition() > inchesToTicks(targetPosition) ||
                    driveMotors[2].getCurrentPosition() > inchesToTicks(targetPosition) ||
                    driveMotors[3].getCurrentPosition() > inchesToTicks(targetPosition))) {

                double power = trapezoidMotionProfile(
                        driveMotors[0].getCurrentPosition(),
                        targetPosition,
                        maxVelocity,
                        (accelerationPeriod * targetPosition)
                );

                setDrivePowers(-power, -power, -power, -power);

                opMode.telemetry.addData("MECANUM DRIVETRAIN: RUNNING TO POSITION %",
                        (1 - (((inchesToTicks(targetPosition) - driveMotors[0].getCurrentPosition()) / inchesToTicks(targetPosition)) * 100)));
                opMode.telemetry.update();
            }
        }

    }

    public void translateX(double targetPosition, double maxVelocity, double accelerationPeriod) {
        /***
         * translateX handles strafing for autonomous driving
         * @param targetPosition target encoder ticks
         * @param maxVelocity maximum motor power the drivetrain should reach
         * @param accelerationPeriod the period of acceleration and deceleration for strafing
         */

        targetPosition = targetPosition * kDistance;

        for(DcMotor dcMotor : driveMotors) {
            dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if(targetPosition > 0) {    // right
            while (!opMode.opModeIsActive() && !opMode.isStopRequested() && (driveMotors[0].getCurrentPosition() < inchesToTicks(targetPosition) ||
                    driveMotors[1].getCurrentPosition() > inchesToTicks(targetPosition) ||
                    driveMotors[2].getCurrentPosition() > inchesToTicks(targetPosition) ||
                    driveMotors[3].getCurrentPosition() < inchesToTicks(targetPosition))) {

                double power = trapezoidMotionProfile(
                        driveMotors[0].getCurrentPosition(),
                        targetPosition,
                        maxVelocity, INITIAL_DRIVE_VELOCITY,
                        (accelerationPeriod * targetPosition)
                );

                setDrivePowers(power, -power, -power, power);

                opMode.telemetry.addData("MECANUM DRIVETRAIN: RUNNING TO POSITION %",
                        (1 - (((inchesToTicks(targetPosition) - driveMotors[0].getCurrentPosition()) / inchesToTicks(targetPosition)) * 100)));
                opMode.telemetry.update();
            }
        } else if(targetPosition < 0) {     // left
            while (!opMode.opModeIsActive() && !opMode.isStopRequested() && (driveMotors[0].getCurrentPosition() > inchesToTicks(targetPosition) ||
                    driveMotors[1].getCurrentPosition() < inchesToTicks(targetPosition) ||
                    driveMotors[2].getCurrentPosition() < inchesToTicks(targetPosition) ||
                    driveMotors[3].getCurrentPosition() > inchesToTicks(targetPosition))) {

                double power = trapezoidMotionProfile(
                        driveMotors[0].getCurrentPosition(),
                        targetPosition,
                        maxVelocity,
                        (accelerationPeriod * targetPosition)
                );

                setDrivePowers(-power, power, power, -power);

                opMode.telemetry.addData("MECANUM DRIVETRAIN: RUNNING TO POSITION %",
                        (1 - (((inchesToTicks(targetPosition) - driveMotors[0].getCurrentPosition()) / inchesToTicks(targetPosition)) * 100)));
                opMode.telemetry.update();
            }
        }

    }

    public void rotate(double targetAngle, double maxVelocity, double accelerationPeriod) {
        /***
         * rotate handles turning for autonomous driving based off of IMU feedback
         * @param targetAngle target angular offset
         * @param maxVelocity maximum motor power the drivetrain should reach
         * @param accelerationPeriod the period of acceleration and deceleration for turning
         */

        targetAngle = targetAngle * kOffset;

        for(DcMotor dcMotor : driveMotors) {
            dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        subsystemIMU.resetAngle();

        if (targetAngle > 0) {      // left
            while (!opMode.opModeIsActive() && !opMode.isStopRequested() && (subsystemIMU.getAngle() < targetAngle)) {

                double power = trapezoidMotionProfile(
                        subsystemIMU.getAngle(),
                        targetAngle,
                        maxVelocity, INITIAL_DRIVE_VELOCITY,
                        (accelerationPeriod * targetAngle)
                );

                setDrivePowers(-power, -power, power, power);
            }
        } else if (targetAngle < 0) {
            while (!opMode.opModeIsActive() && !opMode.isStopRequested() && (subsystemIMU.getAngle() < targetAngle)) {
                double power = trapezoidMotionProfile(
                        subsystemIMU.getAngle(),
                        targetAngle,
                        maxVelocity,
                        (accelerationPeriod * targetAngle)
                );

                setDrivePowers(power, power, -power, -power);
            }
        }
    }

    public void setDrivePowers(double lfPower, double rfPower, double lbPower, double rbPower) {
        /***
         * setDrivePowers handles setting motor powers for drivetrain motors
         * @param lfPower motor power [-1, 1] to be applied to the left front motor
         * @param rfPower motor power [-1, 1] to be applied to the right front motor
         * @param lbPower motor power [-1, 1] to be applied to the left back motor
         * @param rbPower motor power [-1, 1] to be applied to the right back motor
         */

        driveMotors[0].setPower(lfPower);
        driveMotors[1].setPower(lbPower);
        driveMotors[2].setPower(rfPower);
        driveMotors[3].setPower(rbPower);
    }

    public double inchesToTicks(double inches) {
        /***
         * inchesToTicks handles converting inches to encoder ticks
         * @param inches the amount of inches to be converted to encoder ticks
         * @return conversion of inches to motor ticks
         */

        return ((inches * TICKS_PER_REVOLUTION) /
                (2 * Math.PI * (WHEEL_DIAMETER_IN/2)));

    }

    public double ticksToInches(double ticks) {
        /***
         * ticksToInches handles converting encoder ticks to inches
         * @param ticks the amount of ticks to be converted to inches
         * @return conversion of ticks to inches
         */

        return ((ticks * 2 * Math.PI * (WHEEL_DIAMETER_IN/2)) /
                (TICKS_PER_REVOLUTION));

    }

    public double trapezoidMotionProfile(double currentTicks, double trajectoryLength, double maxVelocity, double initialVelocity, double accelerationPeriod) {
        /***
         * trapezoidMotionProfile handles creating a trapezoidal motion profile for driving, turning, and strafing
         * @param currentTicks current encoder ticks or angle of the drivetrain
         * @param trajectoryLength target encoder ticks or angular offset
         * @param maxVelocity maximum motor power the drivetrain should reach
         * @param initialVelocity initial motor power the drivetrain should drive at
         * @param accelerationPeriod the period of acceleration and deceleration for driving, turning, and strafing
         * @return motor power for given position or angle
         */

        if(currentTicks < accelerationPeriod) {
            return (maxVelocity-initialVelocity / accelerationPeriod) * (currentTicks+initialVelocity);
        } else if(currentTicks >= accelerationPeriod && currentTicks <= trajectoryLength-accelerationPeriod) {
            return maxVelocity;
        } else if(currentTicks > trajectoryLength-accelerationPeriod){
            return ((-maxVelocity/accelerationPeriod)*(currentTicks-trajectoryLength));
        }

        return 0;
    }
}
