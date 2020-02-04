package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class CommandRunner {

  private DcMotor fL, fR, bL, bR;
  private Gamepad gp1, gp2;
  
  private MecanumSubsystem driveTrain;
  private DriveCommand driveCommand;

  public CommandRunner(HardwareMap hMap, Gamepad gp1, Gamepad gp2) {
    fL = hMap.get(DcMotor.class, "frontLeft");
    fR = hMap.get(DcMotor.class, "frontRight");
    bL = hMap.get(DcMotor.class, "backLeft");
    bR = hMap.get(DcMotor.class, "backRight");
    
    this.gp1 = gp1;
    this.gp2 = gp2;
  }
  
  public void initialize() {
    driveTrain = new MecanumSubsystem(fL, fR, bL, bR);
    driveCommand = new DriveCommand(
      driveTrain, () -> gp1.left_stick_x,
      () -> gp1.left_stick_y, () -> gp1.right_stick_x);
  }
  
  public void run(boolean isStopRequested) {
    driveCommand.loop(isStopRequested);
  }
  
}
