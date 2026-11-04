// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
//import frc.robot.Constants.OperatorConstants;
import frc.robot.SwerveDrive.SwerveKinematics;
//import frc.robot.commands.Autos;
// import frc.robot.commands.CommandIntake;
// import frc.robot.commands.CommandTurret;
import frc.robot.subsystems.GarraSubsystem;


import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public final GarraSubsystem garra = new GarraSubsystem();
  


 // public static LimelightHelpers limelightHelpers = new LimelightHelpers();



  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  // public static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  // public static TurretSubsystem turretSubsystem = new TurretSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // turretSubsystem.setDefaultCommand(new CommandTurret());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_driverController.rightTrigger().whileTrue(

    Commands.runEnd(

        () -> garra.setMotor(
                m_driverController.getRightTriggerAxis()
        ),

        () -> garra.stopMotor(),

        garra
    )
);
    // m_driverController.getRightTriggerAxis().whileTrue(new Command(Commands.runOnce(() -> garra.setMotor())));
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
        //.onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    // m_driverController.a().onTrue(new Command(Commands.runOnce(()-> intakeSubsystem.SetVel_Colet(0.1))));
      // m_driverController.rightBumper().onTrue(new CommandTurret());


      // m_driverController.rightBumper().toggleOnTrue(new CommandIntake(-10, 1));
      // m_driverController.rightTrigger().whileTrue(new CommandTurret());
      

    
  }
   private void activateCommandOnCondition(BooleanSupplier condition, Command command) {
    new Trigger(condition).onTrue(command);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
  //} public Command getAutonomousCommand() {
    return Commands.none();
}
}

