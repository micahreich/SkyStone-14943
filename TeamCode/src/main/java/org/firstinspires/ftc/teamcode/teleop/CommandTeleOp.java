package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@TeleOp(name="Sample Command-Based")
@Disabled
public class CommandTeleOp extends LinearOpMode {

  private CommandRunner runner;

  @Override
  public void runOpMode throws InterruptedException {
    runner = new CommandRunner(hardwareMap, gamepad1, gamepad2);
    
    runner.initialize();
    
    waitForStart();
    
    while(opModeIsActive()) runner.run(isStopRequested());
  }
  
}
