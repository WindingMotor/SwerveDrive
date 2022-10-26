// FRC2106 Junkyard Dogs - Swerve Drive Base Code

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

// Runs a given pp-trajectory as a command 
public class TrajectoryWeaver extends SequentialCommandGroup{
    
    // Constructor that obtains required values
    public TrajectoryWeaver(SwerveSubsystem swerveSubsystem, PIDController xController,
    PIDController yController,  ProfiledPIDController thetaController,
    PathPlannerTrajectory pptrajectory){

        // Tell theta PID controller that its a circle
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

         // Create path planner controller command, this outputs module states for the pp-trajectory given
        PPSwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(pptrajectory, swerveSubsystem::getPose, DriveConstants.kDriveKinematics, xController, yController, thetaController, swerveSubsystem::setModuleStates, swerveSubsystem);

        // Create report warning command, prints running trajectory to driver station
        ReportWarning sendData = new ReportWarning("Trajectory runner: " + pptrajectory.toString());

        addCommands(
            // Commands to run sequentially
            new SequentialCommandGroup(
                new ResetOdometry(swerveSubsystem, pptrajectory.getInitialPose()),  // Reset robot odometry before movement 
                swerveControllerCommand, // Move robot with trajectory and module states
                sendData, // Tell driver station that command is running
                new InstantCommand(() -> swerveSubsystem.stopModules()) // Stop all modules
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