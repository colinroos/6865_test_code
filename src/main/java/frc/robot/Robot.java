// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.XboxController;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  double DRIVE_POWER = 0.6;
  double INTAKE_POWER = 0.6;

  double armRatio = 50;
  double handRatio = (54 / 12) * 16;

  Timer autoTimer = new Timer();

  XboxController xBox1 = new XboxController(0);

  // Drive motors
  WPI_TalonSRX left = new WPI_TalonSRX(1);
  WPI_TalonSRX leftF = new WPI_TalonSRX(14);
  WPI_TalonSRX right = new WPI_TalonSRX(0);
  WPI_TalonSRX rightF = new WPI_TalonSRX(15);

  MotorControllerGroup leftM = new MotorControllerGroup(left, leftF);
  MotorControllerGroup rightM = new MotorControllerGroup(right, rightF);

  DifferentialDrive diffDrive = new DifferentialDrive(leftM, rightM);

  WPI_TalonSRX intakeSrx = new WPI_TalonSRX(10);
  DigitalInput gamePiece = new DigitalInput(0);

  TalonFX armFx = new TalonFX(4);
  TalonFX handFx = new TalonFX(5);

  int armPos = 1;

  boolean targetSeen;

  public void moveArm(double position) {
    armFx.set(TalonFXControlMode.Position, (int) ((2048 / 360) * position * armRatio));
  }

  public void moveHand(double position) {
    handFx.set(TalonFXControlMode.Position, (int) ((2048 / 360) * position * handRatio));
  }

  public boolean handOnTarget(double position) {
    return ((position * handRatio * (2048 / 360)) - handFx.getSelectedSensorPosition()) < 1000;
  }

  public boolean armOnTarget(double position) {
    return ((position * armRatio * (2048 / 360)) - armFx.getSelectedSensorPosition()) < 1000;
  }

  public void holdArm() {
    if (Math.abs(armFx.getSelectedSensorPosition() - armFx.getClosedLoopTarget()) < 1000) {
      // keep current target
      armFx.set(TalonFXControlMode.Position,
          armFx.getClosedLoopTarget());
    } else {
      // Set target to current position if not on target
      armFx.set(TalonFXControlMode.Position,
          armFx.getSelectedSensorPosition());
    }
  }

  public boolean handOut() {
    // If current position of hand is less than (closer to parellel than) the 70
    // degree clearance
    if (handFx.getSelectedSensorPosition() < (-70 * handRatio * (2048 / 360))) {
      return true;
    } else {
      return false;
    }
  }

  public boolean armIn() {
    // If current position of arm is greater than (closer to perpendicular than) the
    // 85 degree clearance
    if (armFx.getSelectedSensorPosition() > -10 * armRatio * (2048 / 360)) {
      return true;
    } else {
      return false;
    }
  }

  public void holdHand() {
    if (Math.abs(handFx.getSelectedSensorPosition() - handFx.getClosedLoopTarget()) < 1000) {
      handFx.set(TalonFXControlMode.Position,
          handFx.getClosedLoopTarget());
    } else {
      handFx.set(TalonFXControlMode.Position,
          handFx.getSelectedSensorPosition());
    }
  }

  public void armTo(double bicep_position, double forearm_position) {
    if (armIn() && !armOnTarget(bicep_position)) {
      if (handOnTarget(forearm_position)) {
        moveArm(bicep_position);
      } else {
        moveHand(forearm_position);
        holdArm();
      }
    } else if (armOnTarget(bicep_position)) {
      if (!handOut()) {
        moveHand(forearm_position);
      } else {
        moveArm(bicep_position);
        moveHand(forearm_position);
      }
    } else {
      if (handOut()) {
        moveArm(forearm_position);
        holdHand();
      } else
        moveHand(-80);
    }
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    leftM.setInverted(false);
    rightM.setInverted(true);

    armFx.setSelectedSensorPosition(0);
    handFx.setSelectedSensorPosition(handRatio * (2048 / 360));

    armFx.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    armFx.setSensorPhase(true);
    armFx.setInverted(true);
    armFx.config_kF(0, 0);
    armFx.config_kP(0, 0.2);
    armFx.config_kI(0, 0);
    armFx.config_kD(0, 0);
    armFx.configClosedLoopPeakOutput(0, 0.2);
    armFx.configAllowableClosedloopError(0, 0);

    handFx.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    handFx.setSensorPhase(true);
    handFx.setInverted(true);
    handFx.config_kF(0, 0);
    handFx.config_kP(0, 0.4);
    handFx.config_kI(0, 0);
    handFx.config_kD(0, 0);
    handFx.configClosedLoopPeakOutput(0, 0.2);
    handFx.configAllowableClosedloopError(0, 0);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
     // Drive base code 
     if (Math.abs(xBox1.getLeftY())>0.07||Math.abs(xBox1.getLeftX())>0.07) 
     { diffDrive.arcadeDrive(xBox1.getLeftY()*DRIVE_POWER,xBox1.getLeftX()*-DRIVE_POWER);
  
     } 
     else 
     { 
       diffDrive.arcadeDrive(0, 0); 
     } 
     //first drivepower is forward & second drivepower is turn 
     //figure out getY and getX because of button not coords. 
  
  
  
     // Intake - stops rotation based on limiter switch. 
     // Overriden value 
     intakeSrx.set(0); 
     if (xBox1.getLeftTriggerAxis() > 0.2 && !gamePiece.get()) 
     { 
      intakeSrx.set(1); 
     } 
     else if (xBox1.getRightTriggerAxis() > 0.2) 
     { 
       intakeSrx.set(-1); 
     } 
     else  
     { 
       intakeSrx.set(0); 
     } 
      
     //Arm setting - controlled by separate method 
     // A. floor position 
     // B. low shelf 
     // X. top shelf / ??middle post?? 
     // Rt. top post 
     // Default - neutral 
     if (xBox1.getAButton()) 
     { 
       armPos = 1; 
       armTo(-25, -70); 
     } 
     else if (xBox1.getBButton()) 
     { 
       armPos = 2; 
     } 
     else if (xBox1.getXButton()) 
     { 
       armPos = 3; 
     } 
     else if (xBox1.getRightBumper())  
     { 
       armPos = 4; 
     } 
     else 
     {  armPos = 5; 
      armTo(0,0); 
    } 
    // Debug console output 
    System.out.println((armFx.getSelectedSensorPosition())); 
    System.out.println(armIn()); 
    System.out.println(handFx.getClosedLoopTarget()); 
    System.out.println(handOut()); 
     
/* 
 
    // Gyro balance code 
    if (xBox1.getStartButtonPressed()) 
    { 
      if (isBalanced()) 
      { 
        diffDrive.arcadeDrive(0,0); 
      } 
      else 
      { 
        balance(); 
      } 
    }*/ 
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
