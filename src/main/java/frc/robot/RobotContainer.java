// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AimAtHubCommand;
import frc.robot.commands.ShootCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
//import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;

import frc.robot.subsystems.Shooter;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;


import static edu.wpi.first.units.Units.*;

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

  private final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));
  private final Shooter shooter = new Shooter();
  private final Kicker kicker = new Kicker();
  //private final Climber climber = new Climber();
  private final Intake intake = new Intake();
  private final Hood hood = new Hood();

  // private final Superstructure superstructure = new Superstructure(hood, intake, kicker, shooter);
 // private final Pivot pivot = new Pivot();

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

    
    NamedCommands.registerCommand("IntakeBalls", Commands.runOnce(() -> intake.rollerCommand(1).withTimeout(Seconds.of(3))));
    NamedCommands.registerCommand("RevShooter", Commands.runOnce(()-> shooter.setVelocityCommand(RPM.of(2000)).withTimeout(Seconds.of(3))));
    NamedCommands.registerCommand("KickBalls", Commands.runOnce(() -> kicker.feedCommand().withTimeout(Seconds.of(5))));
    NamedCommands.registerCommand("DeployIntake", Commands.runOnce(() -> intake.setPower(.5).withTimeout(Seconds.of(5))));

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
      shooter.setDefaultCommand(shooter.stopCommand());
      intake.setDefaultCommand(intake.setVoltageCommand(Volts.of(0)));
      //climber.setDefaultCommand(climber.StopClimbing());
      
    }

    // :)

    if (Robot.isSimulation()){

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
    // -----------------------------------------------------------------------COMMANDS BEING SET---------------------------------------------------------
    {

      // Driver commands 

      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      
      // Climb

      // driverXbox.leftTrigger().onTrue(climber.ClimbLeft());
      // driverXbox.rightTrigger().onTrue(climber.ClimbRight());

      // driverXbox.leftBumper().onTrue(climber.unClimbLeft());
      // driverXbox.rightBumper().onTrue(climber.unClimbRight());

      // driverXbox.leftTrigger().onFalse(climber.StopClimbing());
      // driverXbox.rightTrigger().onFalse(climber.StopClimbing());

      // driverXbox.leftBumper().onFalse(climber.StopClimbing());
      // driverXbox.rightBumper().onFalse(climber.StopClimbing());


// ---------------------------------------------------------------SHOOTER COMMANDS--------------------------------------------------------------------
    
//------------------------KICKER WHEELS----------------------//
    operatorXbox.rightBumper().onTrue(kicker.feedCommand());
    operatorXbox.rightBumper().onFalse(kicker.stopCommand());


                                                    //------------------------FLYWHEEL COMMAND-----------------------//
                                                    //operatorXbox.rightTrigger().whileTrue(new ShootCommand(shooter, kicker, hood,  Constants.Shooter.hubRPM, Constants.Hood.hubAngle));
                                                    operatorXbox.rightTrigger().whileTrue(shooter.setVelocityCommand(RPM.of(2500)));
                                                    operatorXbox.leftTrigger().whileTrue(shooter.setVelocityCommand(RPM.of(3000)));
                                                    //operatorXbox.povUp().whileTrue(shooter.setVelocityCommand(RPM.of(2300)));
                                                    operatorXbox.povRight().whileTrue(hood.setDegreeCommand(10));
                                                    operatorXbox.povLeft().whileTrue(hood.setDegreeCommand(0));

                                                   

   
                                                    operatorXbox.rightTrigger().whileFalse(shooter.setDutyCycle(0));
                                                    operatorXbox.leftTrigger().whileFalse(shooter.setDutyCycle(0));
                                                    operatorXbox.povDown().whileFalse(shooter.setDutyCycle(0));
                                                    operatorXbox.povLeft().whileFalse(shooter.setDutyCycle(0));

  //--------------------------------------------------------------INTAKE COMMANDS---------------------------------------------------------------

  // ----------PIVOT----------//

  //operatorXbox.leftTrigger().whileTrue(intake.setAngleCommand(Degrees.of(125)));
  //operatorXbox.leftBumper().whileTrue(intake.setAngleCommand(Degrees.of(0)));

// sad backups
operatorXbox.leftTrigger().whileTrue(intake.setPower(.7));
operatorXbox.leftBumper().whileTrue(intake.setPower(-.7));
                                                                                            //---------ROLLERS---------//

                                                                          operatorXbox.b().whileTrue(intake.rollerCommand(1));
                                                                          operatorXbox.b().whileFalse(intake.rollerCommand(0));


// manual hood

operatorXbox.x().whileTrue(hood.setDegreeCommand(5));

 
    
  }

    
    }
   
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // Pass in the selected auto from the SmartDashboard as our desired autonomous commmand 
     return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
