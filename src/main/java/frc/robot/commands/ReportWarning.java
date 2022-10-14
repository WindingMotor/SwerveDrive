
package frc.robot.commands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ReportWarning extends CommandBase {

  private String text;
  private Boolean finished;
  
  public ReportWarning(String text){
      this.text = text;
  }

  @Override
  public void initialize() {
    // Send data to driver station
    DriverStation.reportWarning(text, true);
    // Stop command after we send data to driver station
    finished = true;
  }

  @Override
  public void execute(){}

  @Override
  public boolean isFinished(){
    return finished;
  }

}