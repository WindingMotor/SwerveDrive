
package frc.robot.auto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;

public class testAuto extends SequentialCommandGroup{
    
    public testAuto(SwerveSubsystem drive){
        addCommands(new resetOdometry(drive,Pose2d()));
    }
}
