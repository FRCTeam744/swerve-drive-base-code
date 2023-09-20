// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import org.ejml.data.ZMatrix;





/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {


  public static CTREConfigs ctreConfigs;
  double target = 0.0;
  double current = 0.0;
  private CANSparkMax m_motor;
  private CANSparkMax claw_motor;
  private TalonFX m_arm_motor;
  boolean toggleX = false;
  double clawPower = 0;
  boolean previousXbutton = false;
  Accelerometer accelerometer = new BuiltInAccelerometer();

 // edu.wpi.first.wpilibj.Compressor phCompressor = new  edu.wpi.first.wpilibj.Compressor(0,  edu.wpi.first.wpilibj.PneumaticsModuleType.REVPH);  edu.wpi.first.wpilibj.DoubleSolenoid exampleDoublePCM = new edu.wpi.first.wpilibj.DoubleSolenoid(edu.wpi.first.wpilibj.PneumaticsModuleType.CTREPCM, 0, 1);
  edu.wpi.first.wpilibj.DoubleSolenoid  exampleDoublePH = new edu.wpi.first.wpilibj.DoubleSolenoid(1, edu.wpi.first.wpilibj.PneumaticsModuleType.REVPH, 6, 7);
  
    
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder; 
  double kP = 0.1, kI = 1e-4, kD = 1, kIz = 0, kFF = 0, kMaxOutput = 1, kMinOutput = -1;
  private Command m_autonomousCommand;
  
  private RobotContainer m_robotContainer;
  private final XboxController  m_driverController  = new XboxController(0);
  private final XboxController  m_Controller1  = new XboxController(1);
  
  private final PWMSparkMax Motor = new PWMSparkMax(6);

  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    System.out.println("start here");
    ctreConfigs = new CTREConfigs();
    m_motor = new CANSparkMax(6, MotorType.kBrushless);
    claw_motor = new CANSparkMax(7, MotorType.kBrushless); //claw motor 
    m_arm_motor = new TalonFX(8);

    //m_encoder = m_motor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096); //  getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);
    m_encoder = m_motor.getEncoder(); //  getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);

    //pnumatics 
//    frc::DoubleSolenoid pneumatics{frc::PneumaticsModuleType::REVPH, 0, 1};
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    //System.out.println("go here 1 ");
  }
  
  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}
  
  @Override
  public void disabledPeriodic() {}
  
  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    System.out.println("check here 2");
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }
  
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    


  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    System.out.println("look here 1");
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }
  
  /** This function is called periodically during operator control. */
  @Override
  
  public void teleopPeriodic() {
/*   
   if(m_Controller1.getXButtonPressed() == true){
      target = .3;
      m_motor.set(.3);
    }

    if(m_Controller1.getBButtonPressed() == true){
      target = -.3;
      m_motor.set(-.1);
    }
  if(m_Controller1.getXButtonReleased() || m_driverController.getBButtonReleased()){
      target = 0.05;
      m_motor.set(0.05);
    }
*/
m_arm_motor.set(ControlMode.PercentOutput,(m_Controller1.getLeftY()*.3));
     /*   if(target <= current ){
      current = current - .005;
      }
  if(target >= current){
      current = current + .005;
    }  

if(m_Controller1.getYButtonPressed() == true){
 
  m_arm_motor.set(ControlMode.PercentOutput,.2);
}
if(m_Controller1.getAButtonPressed() == true){

  m_arm_motor.set(ControlMode.PercentOutput,-.2);
}
if((m_Controller1.getYButtonReleased()) || (m_Controller1.getAButtonReleased())){
  m_arm_motor.set(ControlMode.PercentOutput,0);

}
*/ 




m_motor.set(m_Controller1.getRightY() *.5);




// claw intake controls 
/*if(m_Controller1.getRightStickButtonPressed() == true){
  claw_motor.set(.11); //  .set(ControlMode.PercentOutput,.1);
  }
  else if(m_Controller1.getLeftStickButtonPressed() == true){  //  getLeftBumperPressed() == true){
    claw_motor.set(-.11);
  }  
  if(m_Controller1.getRightStickButtonReleased() || m_Controller1.getLeftStickButtonReleased() ){
    claw_motor.set(0);
  }
*/

if((m_Controller1.getXButtonPressed() == true) && (previousXbutton == false)){
  previousXbutton = true;
  if (toggleX == true) {
    clawPower = -.11;
    toggleX = false;
  }
  else {
    clawPower = 0;
    toggleX = true;
  }
}
if(m_Controller1.getXButtonReleased() == true){
  previousXbutton = false;
}  

SmartDashboard.putBoolean("previousXbutton" , previousXbutton);
SmartDashboard.putBoolean("toggle x" , toggleX);
SmartDashboard.putNumber("clawPower", clawPower);
claw_motor.set(clawPower);
  
//pnumatics controls
//  pneumatics.Set(frc::DoubleSolenoid::Value::kReverse);
if(m_Controller1.getRightBumperPressed() == true){
  exampleDoublePH.set(kForward);
//  pneumatics.Set(pneumatics.kForward);
}
if(m_Controller1.getLeftBumperPressed()==true){
//pneumatics.Set(pneumatics.kReverse);
  exampleDoublePH.set(kReverse);
}

if(m_arm_motor.getSelectedSensorPosition(0) <= -40500.0 ){
//  m_arm_motor.set(ControlMode.PercentOutput,0);
}


 


//-----
  }
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
