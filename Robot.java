// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.Time;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  CANSparkMax sparkLeft = new CANSparkMax(10, MotorType.kBrushed); // left wheel motor
  CANSparkMax sparkRight = new CANSparkMax(11, MotorType.kBrushed); // right wheel motor
  CANSparkMax arm2 = new CANSparkMax(20, MotorType.kBrushless); // 20 motor on arm  //  POSITIVE VALUE = going back  //  NEGATIVE = go forward


  VictorSPX Claw = new VictorSPX(1); // for claw 
  VictorSPX arm1 = new VictorSPX(2); // arm on back // hopefully it doesnt die this time


  DigitalInput armLimitSwitch = new DigitalInput(4);

  XboxController driver = new XboxController(0);
  JoystickButton aButton = new JoystickButton(driver, 0);
  JoystickButton xButton = new JoystickButton(driver, 2);
  JoystickButton bButton = new JoystickButton(driver, 1);
  JoystickButton yButton = new JoystickButton(driver, 3);
  JoystickButton rBumperButton = new JoystickButton(driver, 4);
  JoystickButton lBumperButton = new JoystickButton(driver, 5);
  
  Joystick m_stick = new Joystick(0);

  private final DifferentialDrive richard = new DifferentialDrive(sparkRight, sparkLeft);




  // runs claw idk what positive and negative is yet
  void runClaw(){

    boolean lBumperButton = driver.getLeftBumper();
    boolean rBumperButton = driver.getRightBumper();

    double motorspeed = 0.0;

    if (!rBumperButton && !lBumperButton){
       Claw.set(VictorSPXControlMode.PercentOutput, motorspeed);
      return; 
    }
    if ((rBumperButton && lBumperButton)){
    }
    else if (rBumperButton){
      motorspeed = 0.2;
    }
  
    else{
      motorspeed = -0.4;
    }
      Claw.set(VictorSPXControlMode.PercentOutput, motorspeed);
  }






    // runs arm1 idk what positive and whats negative
    void runArm1(){

      boolean aPressed = driver.getAButton();
      boolean bPressed = driver.getBButton();
      boolean limitSwitchState = armLimitSwitch.get();


      double motorspeed = 0.0;
      if (!limitSwitchState && !bPressed){
        arm1.set(VictorSPXControlMode.PercentOutput, motorspeed);
        return;
      }
    
      if (aPressed && bPressed){
        return;
      }
      
      if ((!aPressed && !bPressed)){
      }

      else if (aPressed){
        motorspeed = 0.4;
      }
    
      else{
        motorspeed = -0.4;
      }
      arm1.set(VictorSPXControlMode.PercentOutput, motorspeed);

    }





// running arm2 positive is brings the arm back // negative pushes forward
    void runArm2(){

      boolean xPressed = driver.getXButton();
      boolean yPressed = driver.getYButton();

      double motorspeed = 0.0;

      if (!xPressed && !yPressed){
        arm2.set(motorspeed);
        return;
      }

      if (xPressed && yPressed){
        return;
      }

      else if (xPressed){
        motorspeed = 0.2;
      }

      else{
        motorspeed = -0.2;
      }
        arm2.set(motorspeed);
    }

  boolean motorOn = false;

  



//  ethod for running a motor for a peirod of a time mostly used for tests
public void runMotors() {


    try {
      roboSleep(500);

      motorOn = !motorOn;

      Claw.set(VictorSPXControlMode.PercentOutput, 0.3);


    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }

private static void roboSleep(long millis)
  throws InterruptedException {
  Thread.sleep(millis);
}




  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
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
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    now = System.currentTimeMillis();
    end = now + k_autoRunMillis;
    move = now + timer1;
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  long k_autoRunMillis = 1000;
  long timer1 =3000;  // 3000 for just back 4500 for over charging station 

  long now;
  long end;
  long move;
  


  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    while (end > System.currentTimeMillis()) {
      sparkLeft.set(-0.3);
      sparkRight.set(0.3); 

    
      /* 

      MAIN GOAL OF AUTO:
         Move forward till we hit the edge then put a cone/cube into a node

      */   
    }
 


    while (move > System.currentTimeMillis()){

      sparkLeft.set(0.6);
      sparkRight.set(-0.6);

    }

    sparkLeft.set(0.0);
    sparkRight.set(0.0);
        

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    double y = m_stick.getY();
    double ySpeed = y * 0.8;
    double xSpeed = m_stick.getX() * -0.85;

 


    richard.arcadeDrive(xSpeed, ySpeed);

    runArm1();
    runArm2();
    runClaw();


  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {


  //  runMotors();

  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
