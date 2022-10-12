// File imports
package frc.robot;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
//import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.auto.TestAuto;
import frc.robot.commands.SwerveJoystick;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// GitHub commit version: 10

public class RobotContainer {


  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  // Create joysticks
  private final Joystick leftJoystick = new Joystick(OIConstants.kLeftJoystick);
  private final Joystick rightJoystick = new Joystick(OIConstants.kRightJoystick);

  public RobotContainer() {

    // Set swerve subsystem default command to swerve joystick with respective joystick inputs
    swerveSubsystem.setDefaultCommand(new SwerveJoystick(swerveSubsystem,
    () -> rightJoystick.getRawAxis(0 /* Place axis value here! X-AXIS */),
    () -> rightJoystick.getRawAxis(1 /* Place axis value here! Y-AXIS */),
    () -> leftJoystick.getRawAxis(1 /* Place axis value here! R-AXIS */),
    () -> !leftJoystick.getRawButton(2 /* Place button value here! FIELD ORIENTED? */)));

    configureButtonBindings();
    
  }

  // Create buttons bindings
  private void configureButtonBindings() {

    // Assign button to manually zero heading
    new JoystickButton(rightJoystick,2).whenPressed(() -> swerveSubsystem.zeroHeading());

  }


  private Command testAuto = new TestAuto(swerveSubsystem);
  
  public Command getAutonomousCommand() {

    // Create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveConstants.kDriveKinematics);

    // Generate trajectory
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0,0,new Rotation2d(0)), List.of(
    // Interior points
    new Translation2d(1,0),
    new Translation2d(1,-1)), 
    // Ending point
    new Pose2d(2,-1, Rotation2d.fromDegrees(180)), trajectoryConfig);

    // Create PID controllers for trajectory tracking
    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);

    // Make theta PID controller think its a circle
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Create command to follow trajectory
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory, swerveSubsystem::getPose, DriveConstants.kDriveKinematics, xController, yController, thetaController, swerveSubsystem::setModuleStates, swerveSubsystem);

    // Return auto command to run
    return new SequentialCommandGroup(new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())), swerveControllerCommand, new InstantCommand(() -> swerveSubsystem.stopModules())); // Turn to point at center 
  }












}
