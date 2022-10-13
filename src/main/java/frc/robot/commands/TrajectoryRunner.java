
package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

// Runs a given trajectory as a command 
public class TrajectoryRunner extends SequentialCommandGroup{
    
    // Constructor that obtains required values
    public TrajectoryRunner(SwerveSubsystem swerveSubsystem, PIDController xController,
    PIDController yController,  ProfiledPIDController thetaController,
    Trajectory trajectory, TrajectoryConfig trajectoryConfig){

        // Tell theta PID controller that its a circle
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

         // Create controller command, this outputs module states for the trajectory given
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory, swerveSubsystem::getPose, DriveConstants.kDriveKinematics, xController, yController, thetaController, swerveSubsystem::setModuleStates, swerveSubsystem);

        // Create report warning command
        ReportWarning sendData = new ReportWarning("Trajectory runner: " + trajectory.toString());

        addCommands(
            // Commands to run sequentially
            new SequentialCommandGroup(
                new ResetOdometry(swerveSubsystem, trajectory.getInitialPose()),  // Reset robot odometry before movement 
                swerveControllerCommand, // Move robot
                sendData, // Tell driver station that command is running and the send trajectory name
                new InstantCommand(() -> swerveSubsystem.stopModules()) // Stop modules when move command is done
            )
        ); 

    }
}














/* He be runnin

                        ,////,
                        /// 6|
                        //  _|
                       _/_,-'
                  _.-/'/   \   ,/;,
               ,-' /'  \_   \ / _/
               `\ /     _/\  ` /
                 |     /,  `\_/
                 |     \'
     /\_        /`      /\
   /' /_``--.__/\  `,. /  \
  |_/`  `-._     `\/  `\   `.
            `-.__/'     `\   |
                          `\  \
                            `\ \
                              \_\__
                               \___)
    
*/