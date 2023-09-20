// Author: UMN Robotics Ri3d
// Last Updated : January 2023

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Swerve;
import frc.lib.util.smoothie;
import frc.robot.SwerveModule;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;


import edu.wpi.first.wpilibj2.command.CommandBase;

// This command self=balances on the charging station using gyroscope pitch as feedback
public class BalanceOnBeamCommand extends CommandBase {
  
  Swerve s_Swerve;
  
  private double error;
  private double currentAngle;
  private double drivePower;
  double[] data = new double[100];
  int prd = 50; 
  smoothie ob = new smoothie(prd); 
  Accelerometer accelerometer = new BuiltInAccelerometer();
  SwerveModulePosition[] s_swerveModulePosition;




  /** Command to use Gyro data to resist the tip angle from the beam - to stabalize and balanace */
  public BalanceOnBeamCommand(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Uncomment the line below this to simulate the gyroscope axis with a controller joystick
    // Double currentAngle = -1 * Robot.controller.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS) * 45;
  //this.currentAngle = (s_Swerve.getPitch()).getDegrees(); //-153;
    //this.currentAngle = (s_Swerve.getYaw()).getDegrees(); //-153;
    /*this.currentAngle = (s_Swerve.getRoll()).getDegrees();
    double currentAngleRadians = this.currentAngle * (Math.PI / 180.0);
    drivePower = Math.sin(currentAngleRadians) * -1;
*/
    this.currentAngle = accelerometer.getZ();
    error = Constants.BEAM_BALANCED_GOAL_DEGREES - currentAngle;
    drivePower = currentAngle * 2.5;

    ob.addData(currentAngle);
    
    //drivePower = -Math.min(Constants.BEAM_BALANACED_DRIVE_KP * error, 1);
    
    // Our robot needed an extra push to drive up in reverse, probably due to weight imbalances
    /*if (drivePower < 0) {
      drivePower *= Constants.BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER;
    }*/
    /*   if(Math.abs(drivePower) < .2 ){
        drivePower = .2;
      }*/

      // Limit the max power
      if (Math.abs(drivePower) > 0.4) {
        //drivePower = Math.copySign(0.4, drivePower);
        drivePower = 0.4;
      }
      
      s_Swerve.drive( new Translation2d(0, drivePower),0, false, true);
      
      // Debugging Print Statments
      SmartDashboard.putNumber("Mean", ob.getMean());
      SmartDashboard.putNumber("accel",  accelerometer.getZ());
      SmartDashboard.putNumber("current angle ", currentAngle);
    SmartDashboard.putNumber("error", error);
    SmartDashboard.putNumber("drive power", drivePower);

    //System.out.println("Current Angle: " + currentAngle);
    //System.out.println("Error " + error);
    //System.out.println("Drive Power: " + drivePower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //s_Swerve.drive( new Translation2d(0, 0),0, false, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  //return Math.abs(error) < Constants.BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES; // End the command when we are within the specified threshold of being 'flat' (gyroscope pitch of 0 degrees)
      return Math.abs(error) > 100000;  /// Constants.BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES; // End the command when we are within the specified threshold of being 'flat' (gyroscope pitch of 0 degrees)
     
}
}

