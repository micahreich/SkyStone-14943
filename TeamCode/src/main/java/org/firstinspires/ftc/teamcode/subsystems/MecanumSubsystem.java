package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.Vector2d;

public class MecanumSubsystem {

  private DcMotor[] motors;  // in order of: fL, fR, bL, bR
  
  public MecanumSubsysem(DcMotor... motors) {
    this.motors = motors;
  }
  
  public void disable() {
    for (DcMotor mot : motors) mot.close();
  }
  
  public void stopMotors() {
    for (DcMotor mot : motors) mot.setPower(0);
  }
  
  //
  // THE FOLLOWING METHODS ARE TAKEN FROM FTC LIB:
  // https://github.com/FTCLib/FTCLib/blob/test/ftclib/src/main/java/com/arcrobotics/ftclib/drivebase/MecanumDrive.java
  //
  
  /**
     * Drives the robot from the perspective of the driver. No matter the orientation of the
     * robot, pushing forward on the drive stick will always drive the robot away
     * from the driver.
     *
     * @param xSpeed    the horizontal speed of the robot, derived from input
     * @param ySpeed    the vertical speed of the robot, derived from input
     * @param turnSpeed the turn speed of the robot, derived from input
     */
    public void driveRobotCentric(double xSpeed, double ySpeed, double turnSpeed) {
        driveFieldCentric(xSpeed, ySpeed, turnSpeed, 0.0);
    }

    /**
     * Drives the robot from the perspective of the driver. No matter the orientation of the
     * robot, pushing forward on the drive stick will always drive the robot away
     * from the driver.
     *
     * @param xSpeed    the horizontal speed of the robot, derived from input
     * @param ySpeed    the vertical speed of the robot, derived from input
     * @param turnSpeed the turn speed of the robot, derived from input
     * @param gyroAngle the heading of the robot, derived from the gyro
     */
    public void driveFieldCentric(double xSpeed, double ySpeed,
                                  double turnSpeed, double gyroAngle) {
        Vector2d input = new Vector2d(xSpeed, ySpeed);
        input = input.rotateBy(-gyroAngle);

        double theta = Math.atan2(ySpeed, xSpeed);

        double[] wheelSpeeds = new double[4];
        wheelSpeeds[0 =
                input.magnitude() * Math.sin(theta + Math.PI / 4) + turnSpeed;
        wheelSpeeds[1] =
                input.magnitude() * Math.sin(theta - Math.PI / 4) - turnSpeed;
        wheelSpeeds[2] =
                input.magnitude() * Math.sin(theta - Math.PI / 4) + turnSpeed;
        wheelSpeeds[3] =
                input.magnitude() * Math.sin(theta + Math.PI / 4) - turnSpeed;

        normalize(wheelSpeeds);

        motors[0].setPower(wheelSpeeds[0]);
        motors[1].setPower(-wheelSpeeds[1]);
        motors[2].setPower(wheelSpeeds[2]);
        motors[3].setPower(-wheelSpeeds[3]);
    }
    
    /**
     * Normalize the wheel speeds if any value is greater than 1
     */
    protected void normalize(double[] wheelSpeeds) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        if (maxMagnitude > 1.0) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
            }
        }
    }

}
