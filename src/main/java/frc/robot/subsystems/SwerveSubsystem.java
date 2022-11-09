// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.subsystems;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;



public class SwerveSubsystem extends SubsystemBase {

  // Create 4 swerve modules with attributes from constants
  private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed); 

  // The end of this madness ^_^

  // Create the navX using roboRIO expansion port
  private AHRS gyro = new AHRS(SPI.Port.kMXP);


  // Create odometer for error correction
  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, new Rotation2d(0));

  // Create empty right joystick for live speed control
  Joystick rightJoystick;

  // Swerve subsystem constructor
  public SwerveSubsystem(Joystick rightJoystick) {

    // Assign right joystick
    this.rightJoystick = rightJoystick;

    // Reset navX heading on new thread when robot starts
    new Thread(() -> {
        try {
            Thread.sleep(1000);
            zeroHeading();
        } catch (Exception e) {
        }
    }).start();
  }

  // Reset gyro heading 
  public void zeroHeading() {
    gyro.reset();
  }

  // Return heading in -180* to 180* format
  public double getHeading(){
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }

  // Return heading in Rotation2d format
  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  // Stop all module movement
  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  } 

  public void setModuleStates(SwerveModuleState[] desiredStates) {

    // Make sure robot rotation is all ways possible by changing other module roation speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
}

  // Return robot position caculated buy odometer
  public Pose2d getPose(){
    return odometer.getPoseMeters();
  }

  // Reset odometer to new location
  public void resetOdometry(Pose2d pose){
    odometer.resetPosition(pose, getRotation2d());
  }

  // Periodic looooooop
  @Override
  public void periodic(){

    // Periodicly update odometer for it to caculate position
    odometer.update(getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());

    // Put data on smart dashboard
    SmartDashboard.putNumber("Heading", getHeading());
    SmartDashboard.putString("Field Location", getPose().getTranslation().toString());
    SmartDashboard.putNumber("Front Left ABE: ", frontLeft.getAbsoluteEncoderRad());
    SmartDashboard.putNumber("Front Right ABE: ", frontRight.getAbsoluteEncoderRad());
    SmartDashboard.putNumber("Back Left ABE: ", backLeft.getAbsoluteEncoderRad());
    SmartDashboard.putNumber("Back Right ABE: ", backRight.getAbsoluteEncoderRad());

  }

// Assuming this method is part of a drivetrain subsystem that provides the necessary methods
public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
  // This is just an example event map. It would be better to have a constant, global event map
  // in your code that will be used by all path following commands.
  /* HashMap<String, Command> eventMap = new HashMap<>();
  eventMap.put("marker1", new PrintCommand("Passed marker 1"));
  eventMap.put("intakeDown", new IntakeDown()); */

  return new SequentialCommandGroup(
      new InstantCommand(() -> {
        // Reset odometry for the first path you run during auto
        if(isFirstPath){
            this.resetOdometry(traj.getInitialHolonomicPose());
        }
      }),
      new PPSwerveControllerCommand(traj, this::getPose, DriveConstants.kDriveKinematics, new PIDController(0, 0, 0), new PIDController(0, 0, 0), new PIDController(0, 0, 0), this::setModuleStates, this));
}












}


 

