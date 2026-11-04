// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.SwerveDrive.SwerveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private final SwerveKinematics swerveAngle = new SwerveKinematics();
 // private final SwerveKinematics motorteste = new SwerveKinematics().Garrateste();


  // private SparkMax testeColet = new SparkMax(15, MotorType.kBrushless);
  // private final SparkMax testeIndexerNeo2 = new SparkMax(16, MotorType.kBrushless);
  // private final SparkMax testeIndexerNeo1 = new SparkMax(19, MotorType.kBrushless);
  // // private final TalonFX shutter = new TalonFX(20);

  // private final TalonFX shutter = new TalonFX(17);
  // private final TalonFXConfiguration cfgSutter = new TalonFXConfiguration();
  // private PositionDutyCycle pidShutter = new PositionDutyCycle(0);
  // private final VelocityVoltage pedidoVelocidade = new VelocityVoltage(0).withSlot(0);
  // private double alvoRPS = 0;

  private XboxController controller = new XboxController(0);


  // private TalonFXS Ang_FR = new TalonFXS(6);
  // private TalonFXS Ang_BR = new TalonFXS(8);
  // private TalonFX Ang_FL = new TalonFX(4);

  // private CANcoder CAN_BL = new CANcoder(9);
  // private CANcoder CAN_FL = new CANcoder(10);
  // private CANcoder CAN_BR = new CANcoder(12);
  // private CANcoder CAN_FR = new CANcoder(11);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    
        System.out.println("EstouVivo");
    
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
    swerveAngle.periodic();
    //swerveAngle.PrintsSwerveKnematics();

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

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // cfgmIntake(1, -0.2, 0.2, NeutralModeValue.Brake);
  }
  // private void SetRPM_Shutter(double setRPM) {
  //       this.alvoRPS = setRPM / 60.0;
  //       double meuKv = 0.115;
  //       double VoltagemFeedForward = this.alvoRPS * meuKv;
  //       shutter.setControl(pedidoVelocidade.withVelocity(alvoRPS).withFeedForward(VoltagemFeedForward));
  //   }
  //  private void cfgmIntake(double KP, double OutMin,double OutMax, NeutralModeValue kmode) {
    
  //           cfgSutter.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
  //           cfgSutter.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.5;
  //           cfgSutter.MotorOutput.NeutralMode = kmode;
  //           cfgSutter.CurrentLimits.SupplyCurrentLimit = 40;
  //           cfgSutter.CurrentLimits.StatorCurrentLimitEnable = true;
  //           cfgSutter.Audio.BeepOnConfig = true;
  //           cfgSutter.MotorOutput.PeakForwardDutyCycle = OutMax;
  //           cfgSutter.MotorOutput.PeakReverseDutyCycle = OutMin;
  //           cfgSutter.Voltage.PeakForwardVoltage = 12;
  //           cfgSutter.Voltage.PeakReverseVoltage = -12;
    
  //           Slot0Configs slot0 = cfgSutter.Slot0;
  //           slot0.kP = KP;
  //           slot0.kI = 0.0;
  //           slot0.kD = 0.0;
    
  //           shutter.getConfigurator().apply(cfgSutter);
  //       }
  //       private void setPoint_Intake(double position) {
  //           shutter.setControl(pidShutter.withPosition(position));
  //   }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {


    // if(controller.getRightTriggerAxis() >= 0.2 && controller.getLeftTriggerAxis() < 0.2){
    //   testeIndexerNeo2.set(1);
    //   testeIndexerNeo1.set(-1);
    // }else if(controller.getLeftTriggerAxis() >= 0.2 && controller.getRightTriggerAxis() < 0.2){
    //   testeIndexerNeo2.set(-1);
    //   testeIndexerNeo1.set(1);
    // } else{
    //   testeIndexerNeo2.set(0);
    //   testeIndexerNeo1.set(0);
    // }
    // if(controller.getAButtonPressed()) {
    //   SetRPM_Shutter(6000);
    // }
    // if(controller.getBButtonPressed()) {
    //   SetRPM_Shutter(-6000);
    // }
    // if(controller.getXButtonPressed()) {
    //   shutter.set(0);
    // }
    // if(controller.getRightBumperPressed()) {
    //   setPoint_Intake(30);
    // } else if(controller.getLeftBumperPressed()) {
    //   setPoint_Intake(-30);
    // }
    // if(controller.getAButtonPressed()) {
    //   ola.set(0.4);
    //  }

    //  if(controller.getBButtonPressed()) {
    //    ola.set(-0.8);
    //  }

    // if(controller.getXButtonPressed()) {
    //    ola.set(0.0);
    // }



    // if(controller.getAButtonPressed()) {
    //   // testeColet.set(0);
    //   testeIndexerNeo2.set(0);
    //   testeIndexerNeo1.set(0);
    //   // shutter.set(0);
    // }
    // if(controller.getBButtonPressed()) {
    //   // testeColet.set(-0.6);
    //   // testeIndexerNeo2.set(-1);
    //   // testeIndexerNeo1.set(1);
    //   // shutter.set(0.1);

    // }
    // if(controller.getXButtonPressed()) {
    //   // testeIndexerNeo2.set(1);
    //   // testeIndexerNeo1.set(-1);
    //   // shutter.set(0.4);
    // }
    // if(controller.getYButtonPressed()) {
    //   // testeIndexerNeo2.set(0);
    //   // testeIndexerNeo1.set(0);
    //   // shutter.set(0.1);
    // }
    // if(controller.getLeftBumperButtonPressed()) {
    //   shutter.set(-0.1);

    //   // testeIndexerNeo2.set(0);
    //   // testeIndexerNeo1.set(0);
    // }
    // if(controller.getAButtonPressed()) {
    //   Ang_BR.set(0.1); }
    //   Ang_BL.set(0.1);
    //   Ang_FR.set(0.1);
    //   Ang_FL.set(0.1);
    // }
    // if(controller.getBButtonPressed()) {
    //   Ang_BR.set(0);
    //   Ang_BL.set(0);
    //   Ang_FR.set(0);
    //   Ang_FL.set(0);
    // }
    // SmartDashboard.putNumber("Ang_FL", Ang_FL.getPosition().getValueAsDouble());
    // SmartDashboard.putNumber("Ang_FR", Ang_FR.getPosition().getValueAsDouble());
    // SmartDashboard.putNumber("Ang_BL", Ang_BL.getPosition().getValueAsDouble());
    // SmartDashboard.putNumber("Ang_BR", Ang_BR.getPosition().getValueAsDouble());

    // SmartDashboard.putNumber("CAN_FL", CAN_FL.getAbsolutePosition().getValueAsDouble() * 360);
    // SmartDashboard.putNumber("CAN_FR", CAN_FR.getAbsolutePosition().getValueAsDouble() * 360);
    // SmartDashboard.putNumber("CAN_BL", CAN_BL.getAbsolutePosition().getValueAsDouble() * 360);
    // SmartDashboard.putNumber("CAN_BR", CAN_BR.getAbsolutePosition().getValueAsDouble() * 360);
  }
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}

