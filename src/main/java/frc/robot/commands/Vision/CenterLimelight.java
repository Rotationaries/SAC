// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class CenterLimelight extends CommandBase {
  private Drivetrain m_drive;
  final double GOAL_RANGE_METERS = Units.feetToMeters(3);

  PIDController forwardController = new PIDController(0.5,0,0);
  PIDController turnController = new PIDController(0.5, 0, 0);

  Vision m_camera;
  boolean goalReached;

  double forwardSpeed;
  double turnSpeed;

  private double xPos = m_drive.getPose().getRotation().getDegrees();

  public CenterLimelight(Drivetrain drive, Vision camera) {
    m_drive = drive;
    m_camera = camera;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    forwardController.setTolerance(0.05);
    turnController.setTolerance(0.03);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_camera.getValidTarget() == true){
      double range = m_camera.getDistanceFromCameraToGoal();                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  

      forwardSpeed = -forwardController.calculate(range, GOAL_RANGE_METERS);
      turnSpeed = -turnController.calculate(m_camera.getRY(), 0);
    } else {
      forwardSpeed = 0;
      turnSpeed = 0;
    }

    m_drive.arcadeDrive(forwardSpeed, turnSpeed);

    if(forwardController.atSetpoint()){
      goalReached = true;
      forwardSpeed = 0;
    }

    if(turnController.atSetpoint()){
      turnSpeed = 0;
    }

    if(Math.abs(forwardSpeed) < 0.02){
      forwardSpeed = 0;
    }

    if(Math.abs(turnSpeed) < 0.02){
      turnSpeed = 0;
    }

    // CommandScheduler.getInstance().schedule(new TurnDegrees(LimelightConstants.speed, LimelightConstants.tx, drive));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (forwardController.atSetpoint() && turnController.atSetpoint());
  }
}
