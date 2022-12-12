// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.util;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Transmitter {

    // Joystick object
    // Transmitter Axises: 0 = roll : 1 = pitch : 2 = throttle : 3 = yaw : 4 = analog1 : 5 = analog2
    private Joystick joystick;

    // Class constructor takes: joystick ID
    public Transmitter(int id){

        joystick = new Joystick(id);

    }

    public double calculateOffset(double x){

        if(x > 0){return(1 - x);}
        else if(x < 0){return(-1 - x);}
        else{return(0.0);}
        
    }

    public double getRoll(){
        return(calculateOffset(joystick.getRawAxis(0)));
    }

    public double getPitch(){
        return(calculateOffset(joystick.getRawAxis(1)));
    }



}