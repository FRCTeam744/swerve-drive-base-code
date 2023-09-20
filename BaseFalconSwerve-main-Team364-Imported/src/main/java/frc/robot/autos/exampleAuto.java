package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
//import frc.robot.subsystems.DriveSubsytems;//subsystems.DriveSubsytems;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.BalanceOnBeamCommand;
import frc.robot.commands.DriveForDistanceCommand;


public class exampleAuto extends SequentialCommandGroup {
  private double error;
  private Rotation2d currentAngle;
  private double drivePower;
  public static final double BEAM_BALANACED_DRIVE_KP = 0.001; // P (Proportional) constant of a PID loop
  public static final double BEAM_BALANCED_GOAL_DEGREES = 0;
  public static final double BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES = 1;
  public static final double BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER = 1.15;

  public exampleAuto(Swerve s_Swerve){
System.out.println("switch\n");
   //addCommands(new DriveForDistanceCommand(s_Swerve,5, .7)); // Drive onto the charging station
System.out.println("switch\n");
    addCommands(new BalanceOnBeamCommand(s_Swerve)); // Self-balance on the charging station using Gyroscope pitch as feedback




    System.out.println("auto test");
    double translationVal =  0; //MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        
    double strafeVal = .1;  //MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
    double rotationVal = 0; //MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

    // Drive

   //s_Swerve.drive(
   //       new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
   //       rotationVal * Constants.Swerve.maxAngularVelocity, 
          //!robotCentricSup.getAsBoolean(),
   //       false, 
   //       true
  //);


  //}
//@Override
//public void execute(Swerve s_Swerve) {
  

  this.currentAngle = s_Swerve.getPitch();

  error = BEAM_BALANCED_GOAL_DEGREES - currentAngle.getDegrees();
  drivePower = -Math.min(BEAM_BALANACED_DRIVE_KP * error, 1);

  if (drivePower < 0) {
   drivePower *= BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER;
   }

 // Limit the max power
 if (Math.abs(drivePower) > 0.4) {
   drivePower = Math.copySign(0.4, drivePower);
     }
 if (Math.abs(drivePower) > 0.4) {
   drivePower = Math.copySign(0.4, drivePower);
    }

// s_Swerve.drive( new Translation2d(drivePower, 0),0, false, true);
 
 // Debugging Print Statments

   SmartDashboard.putNumber("Current angle", currentAngle.getDegrees());
   SmartDashboard.putNumber("error", error);
   SmartDashboard.putNumber("drive power", drivePower);

 System.out.println("Current Angle: " + currentAngle);
 System.out.println("Error " + error);
 System.out.println("Drive Power: " + drivePower);

 

//}


  /*   TrajectoryConfig config =
          new TrajectoryConfig(
                  Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                  Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
              .setKinematics(Constants.Swerve.swerveKinematics);

      // An example trajectory to follow.  All units in meters.
      Trajectory exampleTrajectory =
          TrajectoryGenerator.generateTrajectory(
              // Start at the origin facing the +X direction
              new Pose2d(0, 0, new Rotation2d(0)),
              // Pass through these two interior waypoints, making an 's' curve path
              List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
              // End 3 meters straight ahead of where we started, facing forward
              new Pose2d(3, 0, new Rotation2d(0)),
              config);

      var thetaController =
          new ProfiledPIDController(
              Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      SwerveControllerCommand swerveControllerCommand =
          new SwerveControllerCommand(
              exampleTrajectory,
              s_Swerve::getPose,
              Constants.Swerve.swerveKinematics,
              new PIDController(Constants.AutoConstants.kPXController, 0, 0),
              new PIDController(Constants.AutoConstants.kPYController, 0, 0),
              thetaController,
              s_Swerve::setModuleStates,
              s_Swerve);


      addCommands(
          new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
          swerveControllerCommand
      );
*/
  }
}

/*
public class exampleAuto extends SequentialCommandGroup { 

    
    
    

 
                

        double translationVal =  0; //MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        
        double strafeVal = .1;  //MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = 0; //MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        // Drive

      s_Swerve.drive(
              new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
              rotationVal * Constants.Swerve.maxAngularVelocity, 
              /*!robotCentricSup.getAsBoolean(),
              false, 
              true
      );
//@Override
//public void execute() {
  

   this.currentAngle = s_Swerve.getPitch();

   error = BEAM_BALANCED_GOAL_DEGREES - currentAngle.getDegrees();
   drivePower = -Math.min(BEAM_BALANACED_DRIVE_KP * error, 1);

   if (drivePower < 0) {
    drivePower *= BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER;
    }

  // Limit the max power
  if (Math.abs(drivePower) > 0.4) {
    drivePower = Math.copySign(0.4, drivePower);
      }
  if (Math.abs(drivePower) > 0.4) {
    drivePower = Math.copySign(0.4, drivePower);
     }

  s_Swerve.drive( new Translation2d(drivePower, 0),0, false, true);
  
  // Debugging Print Statments

    SmartDashboard.putNumber("Current angle", currentAngle.getDegrees());
    SmartDashboard.putNumber("error", error);
    SmartDashboard.putNumber("drive power", drivePower);

  System.out.println("Current Angle: " + currentAngle);
  System.out.println("Error " + error);
  System.out.println("Drive Power: " + drivePower);

  }
*/
//}

// Called once the command ends or is interrupted.
/*@Override
public void end(boolean interrupted) {
  m_DriveSubsystem.stop();
}
}*/


// Returns true when the command should end.
/*@Override
public boolean isFinished() {
  return Math.abs(error) < BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES; // End the command when we are within the specified threshold of being 'flat' (gyroscope pitch of 0 degrees)
}
 //.drive(drivePower, drivePower);
*/
  //s_Swerve.drive(null, 0, false, true);
 //       addCommands(
 //           new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
 //           swerveControllerCommand
 //       );
  //  }
    