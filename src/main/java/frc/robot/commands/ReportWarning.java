// FRC2106 Junkyard Dogs - Swerve Drive Base Code


package frc.robot.commands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class ReportWarning extends CommandBase {

  // Create unassigned private instance variables
  private String text;
  private Boolean finished;
  
  // Class constructor, assigns text to write
  public ReportWarning(String text){
      this.text = text;
  }

  @Override
  public void initialize() {
    // Send data to driver station
    DriverStation.reportWarning(text, true);
    // Set finished to true
    finished = true;
  }

  // Stop command once finished equals true
  @Override
  public boolean isFinished(){return finished;}

}