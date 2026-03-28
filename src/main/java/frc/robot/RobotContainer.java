// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AimAtHubCommand;
import frc.robot.commands.ShootCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
//import frc.robot.subsystems.Climber;
//import frc.robot.subsystems.Hood;
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

   final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));
  private final Shooter shooter = new Shooter();
  private final Kicker kicker = new Kicker();
  //private final Climber climber = new Climber();
  private final Intake intake = new Intake();


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

    
    NamedCommands.registerCommand("IntakeBalls", intake.rollerCommand(2));

    NamedCommands.registerCommand("RevShooter", shooter.setVelocityCommand(RPM.of(2500)).withTimeout(1));

    NamedCommands.registerCommand("KickBalls", kicker.feedCommand().withTimeout(10));

    NamedCommands.registerCommand("DeployIntake", intake.setPower(.5).withTimeout(1));

    NamedCommands.registerCommand("StopIntaking", intake.rollerCommand(0)); 
    NamedCommands.registerCommand("Stop Shooting and Revving", (shooter.stopCommand().alongWith(kicker.stopCommand()).withTimeout(.5)));

    NamedCommands.registerCommand("REVANDSHOOT",shooter.setVelocityCommand(RPM.of(2500)).andThen(new WaitCommand(3)).deadlineFor(kicker.feedCommand()));

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
      intake.setDefaultCommand(intake.setDutyCycleCommand(0));
      //climber.setDefaultCommand(climber.StopClimbing());
      
    }

    // :

    new Trigger(driverXbox.leftBumper())
    .whileTrue(new RunCommand(
        () -> {
            double forward = LimelightHelpers.getTY("limelight") * -0.3;
            double strafe = driverXbox.getLeftX();
            double rotation = LimelightHelpers.getTX("limelight") * -0.05;

            drivebase.drive(
                new Translation2d(forward, strafe),
                rotation,
                false
            );
        },
        drivebase
    ));
    
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
      // aims the front of the robot at the hub using odometry and the pose of the hub
      driverXbox.rightTrigger().onTrue(new AimAtHubCommand(drivebase, driveAngularVelocity));
      driverXbox.rightTrigger().onFalse(driveFieldOrientedAngularVelocity);
      // shoots the ball based on the distance from the hub using an interpolating tree map!
      driverXbox.leftTrigger().onTrue(new ShootCommand(shooter, kicker, drivebase)); 
      driverXbox.leftTrigger().onFalse(shooter.stopCommand());
     


// ---------------------------------------------------------------SHOOTER COMMANDS--------------------------------------------------------------------
    
//------------------------KICKER WHEELS----------------------//
    operatorXbox.rightBumper().onTrue(kicker.feedCommand());
    operatorXbox.rightBumper().onFalse(kicker.stopCommand());


                                                    //------------------------FLYWHEEL COMMAND-----------------------//
                                                    //operatorXbox.rightTrigger().whileTrue(new ShootCommand(shooter, kicker, hood,  Constants.Shooter.hubRPM, Constants.Hood.hubAngle));
                                                    operatorXbox.rightTrigger().whileTrue(shooter.setVelocityCommand(RPM.of(2500)));
                                                    operatorXbox.povUp().whileTrue(shooter.setVelocityCommand(RPM.of(3000)));
                                                    //operatorXbox.povUp().whileTrue(shooter.setVelocityCommand(RPM.of(2300)));
                                                   

                                                   

   
                                                    operatorXbox.rightTrigger().whileFalse(shooter.setDutyCycle(0));
                                                    operatorXbox.leftTrigger().whileFalse(shooter.setDutyCycle(0));
                                                    operatorXbox.povDown().whileFalse(shooter.setDutyCycle(0));
                                                    operatorXbox.povLeft().whileFalse(shooter.setDutyCycle(0));

                                                    //---------------------UNSTUCK COMMAND--------------//
                                                    operatorXbox.a().onTrue(kicker.backFeedCommand().alongWith(intake.rollerCommand(-1)));
                                                    operatorXbox.a().onFalse(kicker.stopCommand().alongWith(intake.rollerCommand(0)));
                                                

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
