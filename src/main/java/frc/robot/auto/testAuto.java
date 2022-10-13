
package frc.robot.auto;
import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ResetOdometry;
import frc.robot.subsystems.SwerveSubsystem;

// This is a SequentialCommandGroup class we can use this in auto
public class TestAuto extends SequentialCommandGroup{
    
    // Create trajectory settings
    private TrajectoryConfig trajectoryConfig = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveConstants.kDriveKinematics);

//--------------------------------T-R-A-J-E-C-T-O-R-Y---------------------------------------//

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory( /* Starting point*/ new Pose2d(0,0,new Rotation2d(0)), List.of(
    // Interior points
    new Translation2d(1,0),
    new Translation2d(1,-1)), 
    // Ending point
    new Pose2d(2,-1, Rotation2d.fromDegrees(180)), trajectoryConfig);

//--------------------------------T-R-A-J-E-C-T-O-R-Y--------------------------------------------------------//

    // Constructor that set default values and adds commands to run
    public TestAuto(SwerveSubsystem swerveSubsystem, PIDController xController,
    PIDController yController,  ProfiledPIDController thetaController){

        // Make theta PID controller think its a circle
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

         // Create controller command this output module states for the trajectory given
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory, swerveSubsystem::getPose, DriveConstants.kDriveKinematics, xController, yController, thetaController, swerveSubsystem::setModuleStates, swerveSubsystem);
        
        addCommands(
            // Commands to run sequentially
            new SequentialCommandGroup(
                new ResetOdometry(swerveSubsystem, trajectory.getInitialPose()),  // Reset robot odometry before movement 
                swerveControllerCommand, // Command to move robot 
                new InstantCommand(() -> swerveSubsystem.stopModules()) // Stop modules when move command is done
            )
        );
            
            
            
            
            
            
            

   
            
    
            
            
            
    }
}

//  return new SequentialCommandGroup(new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())), swerveControllerCommand, new InstantCommand(() -> swerveSubsystem.stopModules())); // Turn to point at center 