// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.util;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Monitor {

    // Create PDP object
    PowerDistribution PDP = new PowerDistribution();

    // Class constructor
    public Monitor(){}

    public void update(){

        double sysVoltage = PDP.getVoltage();
        double sysCurrent = PDP.getTotalCurrent();
        double sysTemperature = PDP.getTemperature();

        // Check voltage
        if(sysVoltage <= 9){
            DriverStation.reportError("BATTERY -SUPER- CRITICAL: " + sysVoltage, true);
        }
        else if(sysVoltage <= 10){
            DriverStation.reportError("BATTERY CRITICAL: " + sysVoltage , true);
        }
        else if(sysVoltage <= 11){
            DriverStation.reportWarning("BATTERY LOW: " + sysVoltage, true);
        }

        // Check current draw
        if(sysCurrent >= 110){
            DriverStation.reportError("MAX CURRENT DRAW: " + sysCurrent, true);
        }
        else if(sysCurrent >= 60){
            DriverStation.reportWarning("HALF CURRENT DRAW: " + sysCurrent  , true);
        }

        // Check temperature
        if(sysTemperature >= 100){
            DriverStation.reportWarning("PDP TEMP HIGH: " + sysTemperature, true);
        }

        // Report data to smart dashboard
        SmartDashboard.putNumber("Voltage", sysVoltage);
        SmartDashboard.putNumber("Current", sysCurrent);
        SmartDashboard.putNumber("PDP Temperature", sysTemperature);

    }

}