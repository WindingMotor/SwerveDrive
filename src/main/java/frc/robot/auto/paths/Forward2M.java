
package frc.robot.auto.paths;
import frc.robot.auto.util.SwerveTrajectory;

public class Forward2M extends Path{

    private final static double[][] points = {/* Points go here */};

    public SwerveTrajectory getPath(){
        return new SwerveTrajectory(points);
   }
}
