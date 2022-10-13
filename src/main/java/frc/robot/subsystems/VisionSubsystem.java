
package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem extends SubsystemBase{
    
    // Create private instance variables
    NetworkTable table;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;

    // Caculation variables
    private double distance;
    private double x;
    private double y;
    private double a;

    // Constructor
    public VisionSubsystem(){

    // Get limelight from network tables
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    // Set variables from limelight network tables
    // tx and ty are offsets to target in degrees and ta is total target area
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    }

    // Update main position variables
    public void update(){
        x = tx.getDouble(0);
        y = ty.getDouble(0);
        a = ta.getDouble(0);
    }

    public void setView(int v){
        if( v== 0){
            // Set limelight pipeline view to 0
            table.getEntry("pipeline").setNumber(0);
        }else if(v == 1){
            // Set limelight pipeline view to 1
            table.getEntry("pipeline").setNumber(1);
        }
    }

    // Get methods
    public double getX(){
        return(x);
    }

    public double getY(){
        return(y);
    }

    public double getA(){
        return(a);
    }

    public double getDistance(){
        return(distance);
    }







}
