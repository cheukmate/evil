// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter3;
//import frc.robot.subsystems.Shooter3;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Define Controllers.

  final         CommandXboxController driverXbox = new CommandXboxController(0);
  final         CommandXboxController operatorXbox = new CommandXboxController(1);

  // Define Subsystems.

  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));
  private final Shooter3 shooter = new Shooter3();
  private final IntakeWheels test = new IntakeWheels();
  private final Climber climber = new Climber();
  private final Pivot pivot = new Pivot();

  // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing selection of desired auto

   private final SendableChooser<Command> autoChooser;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true)
                                                                               .translationHeadingOffset(true)
                                                                               .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                   0));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer()
  {
    // Configure the trigger bindings

    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    //Create the NamedCommands that will be used in PathPlanner //TODO: Make autos

    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    //Have the autoChooser pull in all PathPlanner autos as options
    autoChooser = AutoBuilder.buildAutoChooser();

    //Set the default auto (do nothing) 
    autoChooser.setDefaultOption("Do Nothing", Commands.none());

    //Add a simple auto option to have the robot drive forward for 1 second then stop
    autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(1));

    //Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    }
  


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);

   
    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
      shooter.setDefaultCommand(shooter.Stop());
      
    }

    // :)

    if (Robot.isSimulation())
    {
    

// simulation pivot commands
     

    // Schedule `setVelocity` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // driverXbox.x().whileTrue(shooter.setVelocity(RPM.of(1000)));
    // driverXbox.y().whileTrue(shooter.setVelocity(RPM.of(10)));
    //driverXbox.y().onTrue(shooter.setVelocity(300));
    // Schedule `set` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // driverXbox.leftBumper().whileTrue(shooter.set(.02));
    // driverXbox.rightBumper().whileTrue(shooter.set(-.02));

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.none());
      driverXbox.start().onTrue(Commands.none());
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());

    } else
    {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());

    // driverXbox.x().whileTrue(shooter.setVelocity(RPM.of(1000)));
    // driverXbox.y().whileTrue(shooter.setVelocity(RPM.of(300)));

    //NOT FINAL
    operatorXbox.x().whileTrue(new InstantCommand(()-> test.Intake(-.6)));
    operatorXbox.x().whileFalse(new InstantCommand(()-> test.Intake(0)));

    //driverXbox.y().onTrue(shooter.setVelocity(150));
    //driverXbox.y().onFalse(shooter.setVelocity(0));


    //shooterclosedloop beta
     
    // operatorXbox.y().onTrue(shooter.setVelocity());
    // operatorXbox.y().onFalse(shooter.Stop());

    driverXbox.y().onTrue(new InstantCommand(()-> shooter.setVelocityVoid(5)));
    driverXbox.y().onFalse(new InstantCommand(()-> shooter.StopVoid()));

    // operatorXbox.b().onTrue(climber.Climb());
    // operatorXbox.b().onFalse(climber.StopClimbing());

    
 //driverXbox.y().whileFalse(new InstantCommand(()-> shooter.stopMotor()));
 //driverXbox.x().whileFalse(new InstantCommand(()-> shooter.stopMotor()));

      //pivot commands, may or may not work lol

      // operatorXbox.a().onTrue(pivot.pivotToAngle(90));
      // operatorXbox.b().onTrue(pivot.Stow());

      operatorXbox.a().onTrue(new InstantCommand(()-> pivot.stupidCommand(-.4)));
      
      operatorXbox.a().onFalse(new InstantCommand(()-> pivot.stupidCommand(0)));
    
    }
   
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // Pass in the selected auto from the SmartDashboard as our desired autnomous commmand 
     return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
