// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Cascade;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.Constants;
//import frc.robot.Constants.CascadeConstants;
import frc.robot.subsystems.Cascade;
import frc.robot.subsystems.Drivetrain;

public class CascadeLevels extends CommandBase {
  /** Creates a new CascadeLevels. */
  private final Cascade m_cascade;
  int level;

  public CascadeLevels(int level, Cascade m_cascade) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.level = level;
    this.m_cascade = m_cascade;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // switch(level){
    //   case 3:


    //   if(m_cascade.getEncoder1Count() == CascadeConstants.pos1);{
    //     m_cascade.stopCascadeMotors();
    //   }
    //   break;
    //   case 2:
    //   if(m_cascade.getEncoder1Count() == CascadeConstants.pos2){
    //     m_cascade.stopCascadeMotors();
    //   }
    //   case 1:
    //   m_cascade.autoCascadeDrive(Constants.CascadeConstants.pos1);

    

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

