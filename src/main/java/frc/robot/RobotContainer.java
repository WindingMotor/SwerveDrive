// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.auto.TestAuto;
import frc.robot.auto.trajectories.Forward2M;
import frc.robot.commands.SwerveJoystick;
import frc.robot.commands.TrajectoryRunner;
import frc.robot.commands.TrajectoryWeaver;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class RobotContainer {

  //------------------------------------V-A-R-I-A-B-L-E-S------------------------------------//

  // Create joysticks
  private final Joystick leftJoystick = new Joystick(IOConstants.kLeftJoystick);
  private final Joystick rightJoystick = new Joystick(IOConstants.kRightJoystick);

  // Create swerve subsystem
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(rightJoystick);

  // Create Xbox controller
  private final XboxController xboxController = new XboxController(IOConstants.kXboxController);

  // Create PID controllers for trajectory tracking
  private final PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
  private final PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
  private final ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);

  //------------------------------------C-O-N-S-T-R-U-C-T-O-R----------------------------//

  public RobotContainer(){

    // Set swerve subsystem default command to swerve joystick with respective joystick inputs
    swerveSubsystem.setDefaultCommand(new SwerveJoystick(swerveSubsystem,
    () -> rightJoystick.getRawAxis(0 /* Place axis value here! X-AXIS */),
    () -> rightJoystick.getRawAxis(1 /* Place axis value here! Y-AXIS */),
    () -> leftJoystick.getRawAxis(1 /* Place axis value here! R-AXIS */),
    () -> !leftJoystick.getRawButton(Constants.IOConstants.kFieldOrientedButton /* Field oriented? */)));

    // Run button binding method
    configureButtonBindings();
  }

  //------------------------------------B-U-T-T-O-N-S------------------------------------//

  // Create buttons bindings
  private void configureButtonBindings(){

    // Assign button to manually zero heading
    new JoystickButton(rightJoystick,Constants.IOConstants.kZeroHeadingButton).whenPressed(() -> swerveSubsystem.zeroHeading());

  }

  //------------------------------------A-U-T-O-N-O-M-O-U-S------------------------------------//
  
  // Create testAuto command without using TrajectoryRunner ;(
  private Command testAuto = new TestAuto(swerveSubsystem, xController, yController, thetaController);

  // Create a command using TrajectoryRunner and passing in trajectory to run
  private Command forward2M = new TrajectoryRunner(swerveSubsystem, xController, yController, thetaController, Forward2M.getTrajectory(), Forward2M.getTrajectoryConfig());


  // Load in trajectory and create command for it
  PathPlannerTrajectory testPath = PathPlanner.loadPath("testPath", new PathConstraints(4, 3) /* velocity and acceleration */ ); 
  private Command PPtest = new TrajectoryWeaver(swerveSubsystem, xController, yController, thetaController, testPath);


  // Returns command to run during auto
  public Command getAutonomousCommand(){

    String autoSelector = "testAuto";
    Command autoCommand = null;

  //------------------------------------S-E-L-E-C-T-O-R------------------------------------//

    // Selector if-statement
    if(autoSelector == "testAuto"){
      autoCommand = testAuto;
    }
    else if(autoSelector == "forward2M"){
      autoCommand = forward2M;
    }

    return autoCommand;
  }
}

