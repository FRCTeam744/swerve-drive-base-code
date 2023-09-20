// Author: UMN Robotics Ri3d
// Last Updated : January 2023

package frc.robot.commands;
import edu.wpi.first.math.geometry.Translation2d;
import java.lang.Math;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// This command drives a specified number of meters
public class DriveForDistanceCommand extends CommandBase {

  Swerve s_Swerve;
  double initialDistance = 0;
  double distance;
  double currentDistance = 0;
  double percentPower;
  double translationVal =  0; //MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);        
  double strafeVal = .1;  //MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
  double rotationVal = 0; //MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
  SwerveModulePosition[] s_swerveModulePosition;
  /** Creates a new DriveForDistanceCommand. */
  public DriveForDistanceCommand(Swerve s_Swerve, double distance, double percentPower) {
    this.s_Swerve = s_Swerve;
    this.distance = distance;
    this.percentPower = percentPower;
    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Drive

//    s_Swerve.drive( new Translation2d(percentPower, .5),0, false, true);
    s_Swerve.drive( new Translation2d(0, -percentPower),0, false, true);
    

  
    
    s_swerveModulePosition = s_Swerve.getModulePositions();


    initialDistance = java.lang.Math.abs(s_swerveModulePosition[0].distanceMeters); 

    SmartDashboard.putNumber("swerve initialDistance",s_swerveModulePosition[0].distanceMeters);
  
 //   System.out.println("INITIAL DISTANCE: " + s_swerveModulePosition[0].distanceMeters);//  initialDistance[0].distance  );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Print statements for debugging
    s_swerveModulePosition = s_Swerve.getModulePositions();
    currentDistance = java.lang.Math.abs(s_swerveModulePosition[0].distanceMeters);
    SmartDashboard.putNumber("currentDistance",currentDistance);
//    SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
    SmartDashboard.putNumber("GOAL DISTANCE", (distance + initialDistance));
    SmartDashboard.putNumber("inital distance", initialDistance);
    SmartDashboard.putNumber("current distance" , currentDistance);
    SmartDashboard.putNumber("power", percentPower);

    System.out.println("GOAL DISTANCE: " + (distance + initialDistance));
    System.out.println("CURRENT DISTANCE: " + currentDistance);
    System.out.println("POWER: " + percentPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    currentDistance = 0;
    s_Swerve.drive( new Translation2d(0, 0),0, false, true); // Stop the rivetrain motors
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return currentDistance >= initialDistance + distance; // End the command when we have reached our goal
    
  }
}