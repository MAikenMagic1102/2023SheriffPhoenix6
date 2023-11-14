// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.GamePiece;
import frc.lib.util.GamePiece.GamePieceType;
import frc.robot.subsystems.CANdleSystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Drive.Swerve;

public class LEDCommand extends CommandBase {
  //Swerve swerve;
  Intake intake;
  CANdleSystem led;
  /** Creates a new LEDCommand. */
  public LEDCommand(Intake intake, CANdleSystem led) {
    //this.swerve = swerve;
    this.intake = intake;
    this.led = led;
    addRequirements(led);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Pose2d currPose = swerve.getVisionPose();
    // if(currPose.getX() < 1.92 && intake.getHasGamepiece()){
    //   led.setColor(0, 0, 255);
    // }else{
      if(intake.getHasGamepiece()){
        led.setColor(0, 255, 0);
      }else{
        if(GamePiece.getGamePiece() == GamePieceType.Cone){
          led.setColor(255, 80, 0);
        }else{
          if(GamePiece.getGamePiece() == GamePieceType.Cube){
            led.setColor(128, 0, 128);
          }
        }
      }
    //}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
