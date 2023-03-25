// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CascadeConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Cascade;
//import frc.robot.subsystems.Cascade;
import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  NetworkTable table = NetworkTableInstance.getDefault().getTable("OI");
    NetworkTableEntry tx = table.getEntry("ForwardBackward");
    NetworkTableEntry ty = table.getEntry("Turning");
    NetworkTableEntry ta = table.getEntry("TurningSens");
  private RobotContainer m_robotContainer;

  private Drivetrain m_drive = new Drivetrain();
  private Cascade m_cascade = new Cascade();
  private Arm m_arm = new Arm();
  private Intake m_intake = new Intake();

  private CANSparkMax  armMotor = new CANSparkMax(8, MotorType.kBrushless);

  private XboxController controller = new XboxController(0);
   private Joystick joystick = new Joystick (1);



  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    setNetworkTablesFlushEnabled(true);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.zeroAllOutputs();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    //m_drive.arcadeDrive(.5, 0);
    
    //  m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // // // // schedule the autonomous command (example)
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.schedule();
    // }


    //PathPlannerTrajectory traj = PathPlanner.loadPath("TestNew", AutoConstants.kMaxSpeedMetersPerSecond, 
    //  AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    //  System.out.println(traj);
      
    //m_cascade.autoCascadeDrive(Constants.CascadeConstants.pos1);*/

    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // System.out.println(m_drive.getPose());
    //m_drive.arcadeDrive(0.5,0);
    //CommandScheduler.getInstance().run();// double x = tx.getDouble(0.0);
    // double y = ty.getDouble(0.0);
    // double area = ta.getDouble(0.0);

    // SmartDashboard.putNumber("ForwardBackward", );
    // SmartDashboard.putNumber("Turning", y);
    // SmartDashboard.putNumber("TurningSens", area);
    
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_drive.arcadeDrive(-controller.getLeftY(), 0.9 * -controller.getRightX());
    // m_drive.controllerMovement(controller);
    //m_cascade.cascadeStagedDrive();
    m_cascade.joyCascade();
    // //m_cascade.autoCascadeDrive(CascadeConstants.pos3FromPos2);
    // //m_cascade.testMotors();
     //m_arm.armDrive();
    m_intake.intakeDrive();

    if (joystick.getRawButton(3)){
      armMotor.set(0.35);
    }
    else if (joystick.getRawButton(4)){
      armMotor.set(-0.15);
    }
    else {armMotor.set(0);}

    
    //m_drive.arcadeDrive(0.5, 0);

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    REVPhysicsSim.getInstance().addSparkMax(Drivetrain.leftMotor1, DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(Drivetrain.leftMotor2, DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(Drivetrain.rightMotor1, DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(Drivetrain.rightMotor2, DCMotor.getNEO(1));
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    double drawCurrent = m_robotContainer.getRobotDrive().getDrawnCurrentAmps();
    double loadedVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(drawCurrent);
    RoboRioSim.setVInVoltage(loadedVoltage);
    REVPhysicsSim.getInstance().run();
  }
}
