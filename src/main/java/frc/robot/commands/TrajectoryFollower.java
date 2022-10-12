
package frc.robot.commands;
import java.nio.file.Path;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.auto.SwerveTrajectory;

// Based off triple helix code
// https://github.com/TripleHelixProgramming/RapidReact/blob/bed3688a07eb925178e7c67a271b4c58336b7fe9/src/main/java/frc/robot/drive/commands/TrajectoryFollower.java#L15

public class TrajectoryFollower extends CommandBase{
    
    private SwerveSubsystem swerveSubsystem;
    private SwerveTrajectory SwerveTrajectory;


    public TrajectoryFollower(SwerveSubsystem swerveSubsystem, Path path) {
        addRequirements(swerveSubsystem);
        this.swerveSubsystem = swerveSubsystem;
        //this.SwerveTrajectory = path.getPath();
      }

}
