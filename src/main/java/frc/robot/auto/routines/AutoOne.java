
package frc.robot.auto.routines;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class AutoOne
{

    public static Command driveAndShoot;

    public static void instantiateAutoCommand(){
        // Create command group and races to run trajectories and methods
        
        driveAndShoot = new SequentialCommandGroup(
            new ParallelCommandGroup( new SequentialCommandGroup() )); 
    }
    
    public static Command getAutoCommand(){
        return driveAndShoot;
    }
}