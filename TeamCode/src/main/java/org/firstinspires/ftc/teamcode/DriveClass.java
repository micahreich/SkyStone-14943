package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class DriveClass {

    RobotHardware robot;

    double width = 18.0; //inches
    int cpr = 537; //counts per rotation
    int gearratio = 1;
    double diameter = 3.94;
    double cpi = (cpr * gearratio)/(Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
    double bias = 0.8;//default 0.8
    double meccyBias = 0.9;//change to adjust only strafing movement

    double TURN_COEFF = 0.8;

    double conversion = cpi * bias;
    Boolean exit = false;

    BNO055IMU imu;
    Orientation angles;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;
    Acceleration gravity;
    LinearOpMode opMode;
    HardwareMap hwm;

    public DriveClass(HardwareMap ahwMap, LinearOpMode aopMode) {
        hwm = ahwMap;

        robot = new RobotHardware();
        robot.initi(ahwMap);

        opMode = aopMode;
    }

    public void initMotors() {
        robot.rightBack.setDirection(DcMotor.Direction.FORWARD);
        robot.rightFront.setDirection(DcMotor.Direction.FORWARD);
        robot.leftBack.setDirection(DcMotor.Direction.REVERSE);
        robot.leftFront.setDirection(DcMotor.Direction.REVERSE);
    }

    public void moveForward(double inches, double speed) {

        int initialPosFL = robot.leftFront.getCurrentPosition();
        int initialPosFR = robot.rightFront.getCurrentPosition();
        int initialPosBL = robot.leftBack.getCurrentPosition();
        int initialPosBR = robot.rightBack.getCurrentPosition();

        int targetPosFL = (int)(initialPosFL + inchToTick(inches));
        int targetPosFR = (int)(initialPosFR + inchToTick(inches));
        int targetPosBL = (int)(initialPosBL + inchToTick(inches));
        int targetPosBR = (int)(initialPosBR + inchToTick(inches));

        double motorSpeed;

        while(!opMode.isStopRequested()) {

            if(     robot.leftFront.getCurrentPosition() < targetPosFL &&
                    robot.rightFront.getCurrentPosition() < targetPosFR &&
                    robot.leftBack.getCurrentPosition() < targetPosBL &&
                    robot.rightBack.getCurrentPosition() < targetPosBR) {

                robot.leftFront.setPower(-speed);
                robot.leftBack.setPower(-speed);
                robot.rightBack.setPower(-speed);
                robot.rightFront.setPower(-speed);

            } else if(robot.leftFront.getCurrentPosition() > targetPosFL ||
                    robot.rightFront.getCurrentPosition() > targetPosFR ||
                    robot.leftBack.getCurrentPosition() > targetPosBL ||
                    robot.rightBack.getCurrentPosition() > targetPosBR) {

                robot.leftFront.setPower(0.0);
                robot.leftBack.setPower(0.0);
                robot.rightBack.setPower(0.0);
                robot.rightFront.setPower(0.0);
                break;

            }
        }

    }

    public void moveBackward(double inches, double speed) {

        int initialPosFL = robot.leftFront.getCurrentPosition();
        int initialPosFR = robot.rightFront.getCurrentPosition();
        int initialPosBL = robot.leftBack.getCurrentPosition();
        int initialPosBR = robot.rightBack.getCurrentPosition();

        int targetPosFL = (int)(initialPosFL - inchToTick(inches));
        int targetPosFR = (int)(initialPosFR - inchToTick(inches));
        int targetPosBL = (int)(initialPosBL - inchToTick(inches));
        int targetPosBR = (int)(initialPosBR - inchToTick(inches));

        double motorSpeed;

        while(!opMode.isStopRequested()) {

            if(     robot.leftFront.getCurrentPosition() > targetPosFL &&
                    robot.rightFront.getCurrentPosition() > targetPosFR &&
                    robot.leftBack.getCurrentPosition() > targetPosBL &&
                    robot.rightBack.getCurrentPosition() > targetPosBR) {

                robot.leftFront.setPower(speed);
                robot.leftBack.setPower(speed);
                robot.rightBack.setPower(speed);
                robot.rightFront.setPower(speed);

            } else if(robot.leftFront.getCurrentPosition() < targetPosFL ||
                    robot.rightFront.getCurrentPosition() < targetPosFR ||
                    robot.leftBack.getCurrentPosition() < targetPosBL ||
                    robot.rightBack.getCurrentPosition() < targetPosBR) {

                robot.leftFront.setPower(0.0);
                robot.leftBack.setPower(0.0);
                robot.rightBack.setPower(0.0);
                robot.rightFront.setPower(0.0);
                break;

            }
        }

    }

    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    public double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    public void rotate(int degrees, double speed)
    {
        resetAngle();

        if(degrees < 0) {
            while(!opMode.isStopRequested()) {
                if(getAngle() > degrees*TURN_COEFF) {
                    move(speed, -speed, -speed, speed);
                } else {
                    move(0, 0, 0, 0);
                    break;
                }
            }
        } else if(degrees > 0) {
            while(!opMode.isStopRequested()) {
                if(getAngle() < degrees*TURN_COEFF) {
                    move(-speed, speed, speed, -speed);
                    opMode.telemetry.addData("GYRO", getAngle());
                    opMode.telemetry.update();
                } else {
                    move(0, 0, 0, 0);
                    break;
                }
            }
        } else {
            return;
        }

        resetAngle();
    }

    public void move(double fr, double fl, double bl, double br) {
        robot.leftFront.setPower(fl);
        robot.leftBack.setPower(bl);
        robot.rightFront.setPower(fr);
        robot.rightBack.setPower(br);
    }

    public double devertify(double degrees){
        if (degrees < 0){
            degrees = degrees + 360;
        }
        return degrees;
    }

    public double convertify(double degrees){
        if (degrees > 179) {
            degrees = -(360 - degrees);
        } else if(degrees < -180) {
            degrees = 360 + degrees;
        } else if(degrees > 360) {
            degrees = degrees - 360;
        }
        return degrees;
    }

    public void initGyro(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "GyroCal.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //
        imu = hwm.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public double inchToTick(double inch) {
        return inch * cpi;
    }

}