// File imports
package frc.robot;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


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
    () -> -leftJoystick.getRawAxis(0 /* Place axis value here! X-AXIS */)
    () -> -leftJoystick.getRawAxis(0 /* Place axis value here! Y-AXIS */)
    () -> -leftJoystick.getRawAxis(0 /* Place axis value here! R-AXIS */)
    () -> !leftJoystick.getRawButton(0 /* Place button value here! FIELD ORIENTED? */)))

    configureButtonBindings();
    
  }
  
  // Create buttons bindings
  private void configureButtonBindings() {

    // Assign button to manually zero heading
    new JoystickButton(rightJoystick,0).whenPressed(() -> swerveSubsystem.zeroHeading())

  }


  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }












}
