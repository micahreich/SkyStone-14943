package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommand;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.util.PIDController;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class CommandRunner {

    private DcMotor fL, fR, bL, bR, liftL, liftR, intakeL, intakeR;
    private Gamepad gp1, gp2;
    private DistanceSensor baySensor;

    PIDController pidController;

    private MecanumSubsystem driveTrain;
    private DriveCommand driveCommand;
    private Lift lift;
    private LiftCommand liftCommand;
    private Intake intake;
    private IntakeCommand intakeCommand;

    public CommandRunner(HardwareMap hMap, Gamepad gp1, Gamepad gp2) {
      fL = hMap.get(DcMotor.class, "frontLeft");
      fR = hMap.get(DcMotor.class, "frontRight");
      bL = hMap.get(DcMotor.class, "backLeft");
      bR = hMap.get(DcMotor.class, "backRight");

      liftL = hMap.get(DcMotor.class, "liftLeft");
      liftR = hMap.get(DcMotor.class, "liftRight");

      intakeL = hMap.get(DcMotor.class, "intakeLeft");
      intakeR = hMap.get(DcMotor.class, "intakeRight");

      baySensor = hMap.get(DistanceSensor.class, "stoneDetector");

      this.gp1 = gp1;
      this.gp2 = gp2;

      pidController = new PIDController(0.02,1,1);
    }

    public void initialize() {
        driveTrain = new MecanumSubsystem(fL, fR, bL, bR);
        driveCommand = new DriveCommand(
                driveTrain
        );
        lift = new Lift(liftL, liftR, pidController);
        liftCommand = new LiftCommand(
                lift
        );
        intake = new Intake(intakeL, intakeR, baySensor);
        intakeCommand = new IntakeCommand(
                intake
        );
    }

    public void run(boolean isStopRequested) {
      driveCommand.loop(isStopRequested,
                        gp1.left_stick_x,
                        gp1.left_stick_y,
                        gp1.right_stick_x);
      if (gp2.a) {
        lift.moveLiftUpToNextStoneLevel();
      }
      if(gp1.a) {
          intakeCommand.loop();
      }
      liftCommand.loop(gp2.right_stick_y);
    }

}
