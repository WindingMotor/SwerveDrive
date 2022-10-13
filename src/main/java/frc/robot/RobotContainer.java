
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
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
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

  // Create PID controllers for trajectory tracking
  private final PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
  private final PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
  private final ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);


  public RobotContainer(){

    // Set swerve subsystem default command to swerve joystick with respective joystick inputs
    swerveSubsystem.setDefaultCommand(new SwerveJoystick(swerveSubsystem,
    () -> rightJoystick.getRawAxis(0 /* Place axis value here! X-AXIS */),
    () -> rightJoystick.getRawAxis(1 /* Place axis value here! Y-AXIS */),
    () -> leftJoystick.getRawAxis(1 /* Place axis value here! R-AXIS */),
    () -> !leftJoystick.getRawButton(2 /* Place button value here! FIELD ORIENTED? */)));

    configureButtonBindings();
  }

  // Create buttons bindings
  private void configureButtonBindings(){

    // Assign button to manually zero heading
    new JoystickButton(rightJoystick,2).whenPressed(() -> swerveSubsystem.zeroHeading());
  }


  // Create testAuto command
  private Command testAuto = new TestAuto(swerveSubsystem, xController, yController, thetaController);
  
  public Command getAutonomousCommand() {

    Command autoCommand = null;

    autoCommand = testAuto;

    return autoCommand;

  }












}
