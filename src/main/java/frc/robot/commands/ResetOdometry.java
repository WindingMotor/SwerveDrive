package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class ResetOdometry extends CommandBase {

  private Pose2d pose;
  private SwerveSubsystem swerveSubsystem;

  public ResetOdometry(SwerveSubsystem swerveSubsystem, Pose2d pose){
    this.swerveSubsystem = swerveSubsystem;
    this.pose = pose;
  }

  @Override
  public void initialize() {
    swerveSubsystem.resetOdometry(pose);
  }

  @Override
  public void execute(){}

  @Override
  public boolean isFinished(){
    return true;
  }

}