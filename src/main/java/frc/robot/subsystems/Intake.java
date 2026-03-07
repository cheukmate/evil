
package frc.robot.subsystems;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

import yams.motorcontrollers.local.SparkWrapper;

public class Intake extends SubsystemBase {

  private static final double INTAKE_SPEED = 1;

  // ThriftyNova controlling the intake roller
  private SparkFlex roller = new SparkFlex(Constants.IDConstants.INTAKEWHEELS_FLEX_MAIN, MotorType.kBrushless);

  private SmartMotorControllerConfig rollerConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      .withTelemetry("IntakeRollerMotor", TelemetryVerbosity.LOW)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1))) // Direct drive, adjust if geared
      .withMotorInverted(false) 
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40));

  private SmartMotorController rollerSmartMotorController = new SparkWrapper(roller, DCMotor.getNeoVortex(1), rollerConfig);

  private final FlyWheelConfig intakeConfig = new FlyWheelConfig(rollerSmartMotorController)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(0.5))
      .withUpperSoftLimit(RPM.of(6000))
      .withLowerSoftLimit(RPM.of(-6000))
      .withTelemetry("IntakeRoller", TelemetryVerbosity.LOW);

  private FlyWheel intake = new FlyWheel(intakeConfig);

  public void Config(){
  
    final SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(25, 0, 0)  // TODO: TUNE TS
    .outputRange(-1, 1)
    .maxMotion
            .cruiseVelocity(3)
            .maxAcceleration(0.35)
            .allowedProfileError(1.0)
            .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
    pivotConfig.softLimit
    .forwardSoftLimit(100.0)
    .reverseSoftLimit(-5.0)
    .forwardSoftLimitEnabled(true)
    .reverseSoftLimitEnabled(true);

    pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pivotMotor.getEncoder().setPosition(0);
  }

  // 5:1, 5:1, 60/18 reduction
   private SmartMotorControllerConfig intakePivotSmartMotorConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(25, 0, 0, DegreesPerSecond.of(360), DegreesPerSecondPerSecond.of(360)) // change THis,base of 25
      .withFeedforward(new SimpleMotorFeedforward(0, 10, 0)) // change, base of 10
     // .withTrapezoidalProfile(RPM.of(3), RPM.of(.35)) 
     .withTelemetry("IntakePivotMotor", TelemetryVerbosity.HIGH)
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 4, 4, 2.18))) // 5, 4, 4, 2.18 (hope it works lol)
     //.withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 5, 60.0 / 18.0, 42)))
      .withMotorInverted(true)
      .withIdleMode(MotorMode.BRAKE)
      
     .withSoftLimit(Degrees.of(0), Degrees.of(150)) //make real number
    .withStatorCurrentLimit(Amps.of(10))
 .withClosedLoopRampRate(Seconds.of(0.1))
      
.withOpenLoopRampRate(Seconds.of(0.1));
    
    //???

  private SparkMax pivotMotor = new SparkMax(Constants.IDConstants.PIVOT, MotorType.kBrushless);

  private SmartMotorController intakePivotController = new SparkWrapper(pivotMotor, DCMotor.getNEO(1),
      intakePivotSmartMotorConfig);

  private final ArmConfig intakePivotConfig = new ArmConfig(intakePivotController)
      .withSoftLimits(Degrees.of(0), Degrees.of(150)) //make real number
      .withHardLimit(Degrees.of(0), Degrees.of(155)) // make real number
      .withStartingPosition(Degrees.of(0))
      .withLength(Feet.of(1))
      .withMass(Pounds.of(2)) // Reis says: 2 pounds, not a lot
    
      .withTelemetry("IntakePivot", TelemetryVerbosity.HIGH);

  private Arm intakePivot = new Arm(intakePivotConfig);

  public Intake() {
    // pivotMotor.factoryReset();
  
  }

  /**
   * Command to run the intake while held.
   */
  public Command intakeCommand() {
    return intake.set(INTAKE_SPEED);//.finallyDo(() -> rollerSmartMotorController.setDutyCycle(0)).withName("Intake.Run");
  }

  public Command Stop(){
    return intake.set(0);
  }

  /**
   * Command to eject while held.
   */
  public Command ejectCommand() {
    return intake.set(-INTAKE_SPEED).finallyDo(() -> rollerSmartMotorController.setDutyCycle(0)).withName("Intake.Eject");
  }

  public Command setPivotAngle(Angle angle) {
    return intakePivot.setAngle(angle).withName("IntakePivot.SetAngle");
  }

  public Command rezero() {
    return Commands.runOnce(() -> pivotMotor.getEncoder().setPosition(0), this).withName("IntakePivot.Rezero");
  }

  /**
   * Command to deploy intake and run roller while held.
   * Stops roller when released.
   */
  public Command deployAndRollCommand() {
    return Commands.run(() -> {
      setIntakeDeployed();
      //rollerSmartMotorController.setDutyCycle(INTAKE_SPEED);
    }, this).finallyDo(() -> {
      //rollerSmartMotorController.setDutyCycle(0);
      setIntakeHold();
    }).withName("Intake.DeployAndRoll");
  }

  public Command deployCommand(){
    return Commands.run(() -> {
      setIntakeDeployed();
    });


  }

  public Command Deploy(){
    return Commands.run(() -> {
      setIntakeDeployed();
    });
  }

   public Command Stow(){
    return Commands.run(() -> {
      setIntakeStow();
    });
  }

  public Command backFeedAndRollCommand() {
    return Commands.run(() -> {
      setIntakeDeployed();
      rollerSmartMotorController.setDutyCycle(-INTAKE_SPEED);
    }, this).finallyDo(() -> {
      rollerSmartMotorController.setDutyCycle(0);
      setIntakeHold();
    }).withName("Intake.BackFeedAndRoll");
  }

  private void setIntakeStow() {
    intakePivotController.setPosition(Degrees.of(0));
  }

  private void setIntakeFeed() {
    intakePivotController.setPosition(Degrees.of(59));
  }


  private void setIntakeHold() {
    intakePivotController.setPosition(Rotations.of(0.348));
  }

  private void setIntakeDeployed() {
    intakePivotController.setPosition(Rotations.of(0.348));// shange
  }



  @Override
  public void periodic() {
    intake.updateTelemetry();
    intakePivot.updateTelemetry();
   
  }

  @Override
  public void simulationPeriodic() {
    intake.simIterate();
    intakePivot.simIterate();
  }
}
