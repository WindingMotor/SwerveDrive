
package frc.robot.subsystems;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import edu.wpi.first.hal.simulation.DIODataJNI;
import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.estimator.AngleStatistics;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
 
  // Create empty variables for reassignment
  private final CANSparkMax driveMotor;
  private final CANSparkMax turningMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turningEncoder;

  private final PIDController turningPidController;

  private final AnalogInput absoluteEncoder;
  private final boolean absoluteEncoderReversed;
  private final double absoluteEncoderOffsetRad;


  // Class constructor where we assign default values for variables
   public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoLuteEncoderReversed) {

    // Create and set offsets and reverse state for encoders
    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.absoluteEncoderReversed = absoLuteEncoderReversed;
    absoluteEncoder = new AnalogInput(absoluteEncoderId);

    // Create drive and turning motor
    driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
    turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

    // Set reverse state of drive and turning motor
    driveMotor.setInverted(driveMotorReversed);
    turningMotor.setInverted(turningMotorReversed);

    // Set drive and turning motor encoder values
    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turningMotor.getEncoder();

    // Change drive motor conversion factors
    driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
    driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);

    // Change drive turning conversion factors
    turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
    turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

    // Create PID controller
    turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);

    // Tell PID controller that it is a *wheel*
    turningPidController.enableContinuousInput(-Math.PI, Math.PI);

    // Call resetEncoders method to set turning encoder to match absolute encoder value
    resetEncoders();

  }

  // Helpful get methods
  public double getDrivePosition() {
    return driveEncoder.getPosition();
  }

  public double getTurningPosition() {
      return turningEncoder.getPosition();
    }

  public double getDriveVelocity() {
      return driveEncoder.getVelocity();
    }

  public double getTurningVelocity() {
      return turningEncoder.getVelocity();
    }

  // Get the swerve module absolute encoder value for other methods
  public double getAbsoluteEncoderRad(){
    double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
    angle *= 2.0 * Math.PI;
    angle -= absoluteEncoderOffsetRad;
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
  }

  // Set turning encoder to match absolute encoder value
  public void resetEncoders(){
    driveEncoder.setPosition(0);
    turningEncoder.setPosition(getAbsoluteEncoderRad());
  }

  // Get swerve module current state, aka velocity and wheel rotation
  public SwerveModuleState getState(){
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  public void setDesiredState(SwerveModuleState state){
    
    // Check if new command has high driving velocity 
    if(Math.abs(state.speedMetersPerSecond) < 0.001){
      stop();
      return;
    }

    // Optimize swerve module state to do fastest rotation movement, aka never rotate more than 90*
    state = SwerveModuleState.optimize(state, getState().angle);

    // Scale velocity down using robot max speed
    driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

    // Use PID to calculate angle setpoint
    turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));

    // Output debugging information to smart dashboard
    SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());

  }

  // STOP STOP STOP STOP STOP STOP STOP STOP STOP STOP STOP STOP STOP STOP STOP, why is it called STOP thats a weird word.
  public void stop() {
    driveMotor.set(0);
    turningMotor.set(0);

  }



  
}
