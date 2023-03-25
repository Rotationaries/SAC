// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import javax.swing.filechooser.FileNameExtensionFilter;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
//import frc.robot.commands.Autonomous.FollowingPath;
//import frc.robot.commands.Autonomous.CreatingPaths;
//import frc.robot.commands.Autonomous.FollowingPath;
// import frc.robot.commands.Drive.Drive;

import frc.robot.subsystems.Drivetrain;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_robotDrive = new Drivetrain();
  //private final CreatingPaths m_pathCreator = new CreatingPaths(m_robotDrive);
  private PathPlannerTrajectory path = new PathPlannerTrajectory();
  private Map<String, Command> eventMap = new HashMap<>();
  public PIDController x = new PIDController(0.2, 0, 0);
  public PIDController y = new PIDController(0.2,0,0);
  private List<PathPlannerTrajectory> alist = new ArrayList<>();
  private List<Command> clist = new ArrayList<>();
  private String fileName;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Containssubsystems, OI devices, and commands. */
  public RobotContainer(/*String fileName*/) {
    // Configure the trigger bindings
    alist = PathPlanner.loadPathGroup(fileName, AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);

    this.fileName = fileName;
    // configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  // private void configureBindings() {
  //   // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  //   new Trigger(m_robotDrive::exampleCondition)
  //       .onTrue(new Drive(m_robotDrive));

  //   // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
  //   // cancelling on release.
  //   m_driverController.y().whileTrue(m_robotDrive.exampleMethodCommand());
  //   m_driverController.b().whileTrue(m_robotDrive.getBalance());
  // }

  public Drivetrain getRobotDrive() {
    return m_robotDrive;
  }

  public void zeroAllOutputs() {
    m_robotDrive.tankDriveVolts(0, 0);
    
  }

  public void configureAuto() {
    eventMap.put("turn1", new PrintCommand("passed marker 1"));
    eventMap.put("turn2", new PrintCommand("passed marker 2"));
    eventMap.put("turn3", new PrintCommand("passed marker 3"));
    eventMap.put("turn4", new PrintCommand("passed marker 4"));

    // for(int i=0; i<alist.size(); i++) {
    //   clist.add(new FollowPathWithEvents(new FollowingPath(alist.get(i), fileName), alist.get(i).getMarkers(), eventMap));
    // }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return new CreatingPaths(m_robotDrive, "DirectDock-M");
    //return m_pathCreator.dockPath();
    //return new DriveDistance(0.5, m_robotDrive);
    //PathPlannerTrajectory traj = PathPlanner.loadPath("1CO1CU-B",new PathConstraints(2, 1.5));
    
    PathPlannerTrajectory traj = PathPlanner.loadPath("TestNew", AutoConstants.kMaxSpeedMetersPerSecond, 
      AutoConstants.kMaxAccelerationMetersPerSecondSquared);
      System.out.println(traj);
    PPRamseteCommand command = new PPRamseteCommand(traj, m_robotDrive::getPose, new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta), 
    AutoConstants.feedforward, DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds, 
    x, y, m_robotDrive::tankDriveVolts, m_robotDrive); 
  // ArrayList<PathPoint> pointlist = new ArrayList<PathPoint>(Arrays.asList(new PathPoint(new Translation2d(1.0,1.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0))));

    
  return command.andThen(() -> m_robotDrive.tankDriveVolts(0,0));


  /*path = PathPlanner.loadPath("TestPath2", new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));
  //paths = PathPlanner.loadPathGroup("1CO", AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
  return new SequentialCommandGroup(
    new WaitCommand(4),
    new FollowPathWithEvents(new FollowingPath(path), path.getMarkers(), eventMap)*/

  /*return new SequentialCommandGroup(
    new WaitCommand(4),
    clist.split());*/
  }
}

