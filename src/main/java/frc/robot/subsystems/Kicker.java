package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class Kicker extends SubsystemBase {


  private static final double KICKER_SPEED = .9;

 
  private SparkMax kicker = new SparkMax(Constants.IDConstants.KICKER, MotorType.kBrushless); // here's the kicker

  private SmartMotorControllerConfig kickerSmartMotorControllerConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.OPEN_LOOP)
  .withGearing(new MechanismGearing(GearBox.fromReductionStages(1))) // probably no gearbox
  .withMotorInverted(true)
  .withTelemetry("Kicker Wheels", TelemetryVerbosity.HIGH)
  .withIdleMode(MotorMode.COAST)
  .withStatorCurrentLimit(Amps.of(40));

  private SmartMotorController kickerSmartMotorController = new SparkWrapper(kicker, DCMotor.getNEO(1), kickerSmartMotorControllerConfig);

  private final FlyWheelConfig kickerConfig = new FlyWheelConfig(kickerSmartMotorController)
  .withDiameter(Inches.of(4))
  .withMass(Pounds.of(0.5))
  .withUpperSoftLimit(RPM.of(3000))
  .withLowerSoftLimit(RPM.of(-3000))
  .withTelemetry("kicker", TelemetryVerbosity.HIGH);

  private FlyWheel kickerWheels = new FlyWheel(kickerConfig);
  

  public Kicker() {
  }

  /**
   * Command to push balls into the shooter while ran.
   */
  public Command feedCommand() {
    return kickerWheels.set(-KICKER_SPEED).finallyDo(() -> kickerSmartMotorController.setDutyCycle(0));
    
  }

  public Command backFeedCommand() {
    return kickerWheels.set(KICKER_SPEED).finallyDo(() -> kickerSmartMotorController.setDutyCycle(0));

  }

  public void setSpeed(double speed){
    System.out.println("setting speed to..." + speed);
    kickerWheels.set(speed);
  }


  /**
   * Command to run the hopper in reverse while held.
   */
 
  /**
   * Command to stop the hopper.
   */
  public Command stopCommand() {
    return kickerWheels.set(0);
  }

  @Override
  public void periodic() {
   
  }

  @Override
  public void simulationPeriodic() {
    kickerWheels.simIterate();
  }
}