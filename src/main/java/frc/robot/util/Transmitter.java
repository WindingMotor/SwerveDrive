// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.util;
import java.text.DecimalFormat;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Transmitter {

    // Joystick object
    // Transmitter Axises: 0 = roll : 1 = pitch : 2 = throttle : 3 = yaw : 4 = analog1 : 5 = analog2
    //roll: 0.700 and -0.700
    private Joystick joystick;
    DecimalFormat df = new DecimalFormat("###.###");

    // Class constructor takes: joystick ID
    public Transmitter(int id){

        joystick = new Joystick(id);

    }

    public double calculateOffset(double d){
        double x =  Double.parseDouble(df.format(d));
        if(x > 0.000){System.out.println(">0: " + x); return(x*(1 - x));}
        if(x <  0.000){System.out.println("<0: " + x); return(x*(-1 - x));}
        else{System.out.println("=0: " + x); return( 0.000);}
        
    }

    public double getRoll(){
        return(calculateOffset(joystick.getRawAxis(0)));
    }

    public double getPitch(){
        return(calculateOffset(joystick.getRawAxis(1)));
    }



}