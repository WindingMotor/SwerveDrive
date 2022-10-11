
package frc.robot.auto;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.routines.AutoOne;

public class AutoSelector
{

    public static Command getAutoCommand()
    {
        //return a commnd from a routine
        return AutoOne.getAutoCommand();
    }
}