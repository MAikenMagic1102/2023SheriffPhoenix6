// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.Arm;

public class StowArm extends CommandBase {
  /** Creates a new ArmToNode. */
  private Arm arm;
  private Intake intake;

  private double lowerArmSetpoint = 0.0;
  private double upperArmSetpoint = 0.0;

  private boolean isFirstRun = true;
  private boolean isFinished = false;

  int i = 0;

  public StowArm(Arm arm, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.intake = intake;
    addRequirements(intake, arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(intake.getHasGamepiece()){
      lowerArmSetpoint = Constants.Arm.STOW.lowerArmSetpoint;
      upperArmSetpoint = Constants.Arm.STOW.upperArmSetpoint;
    }else{
      intake.intakeStop();
      lowerArmSetpoint = Constants.Arm.NOGAMESTOW.lowerArmSetpoint;
      upperArmSetpoint = Constants.Arm.NOGAMESTOW.upperArmSetpoint;
    }
    isFirstRun = true;
    //System.out.println("IsFirstRun: " + isFirstRun);
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(isFirstRun){
      arm.setUpperArmSetPoint(Constants.Arm.RETRACT);
      //System.out.println("Upper Arm is set to : " + Constants.Arm.RETRACT);
      isFirstRun = false;
      //System.out.println("IsFirstRun: " + isFirstRun);
    }


    if(Math.abs(Constants.Arm.RETRACT - arm.getUpperArmPosition()) < 0.3){
      arm.setLowerArmSetPoint(lowerArmSetpoint);
      //System.out.println("Lower Arm is set to : " + lowerArmSetpoint);
    }

    if(Math.abs(lowerArmSetpoint - arm.getLowerArmPosition()) < 0.3){
      arm.setUpperArmSetPoint(upperArmSetpoint);
      //System.out.println("Upper Arm is set to : " + upperArmSetpoint);
    }


    if((Math.abs(upperArmSetpoint - arm.getUpperArmPosition()) < 0.3) && (Math.abs(lowerArmSetpoint - arm.getLowerArmPosition()) < 0.3)){
        //System.out.println("Complete Exiting Command");
        isFinished = true;
      }  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isFinished = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
