
package frc.robot.auto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ResetOdometry;
import frc.robot.subsystems.SwerveSubsystem;

public class TestAuto extends SequentialCommandGroup{
    
    SwerveSubsystem swerveSubsystem;

    public TestAuto(SwerveSubsystem swerveSubsystem){
        this.swerveSubsystem = swerveSubsystem;

        addCommands(
            
            
           // new Pose2d(new Translation2d(-0.7, 0), Rotation2d.fromDegrees(-90.0))
            
            
            );
    }
}
