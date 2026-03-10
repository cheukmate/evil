// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class Hood extends SubsystemBase {
 
  // 1 Neo550, blank degree variability, blank reduction
  private SparkMax hoodSpark = new SparkMax(Constants.IDConstants.SHOOTERHOOD,
   MotorType.kBrushless);

  private SmartMotorControllerConfig smcConfig = new
   SmartMotorControllerConfig(this)
   .withControlMode(ControlMode.CLOSED_LOOP)
   .withClosedLoopController(100, 0, 0, DegreesPerSecond.of(90),
   DegreesPerSecondPerSecond.of(90))
   .withFeedforward(new ArmFeedforward(0, 0.3, 0.1))
   .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
   .withGearing(new MechanismGearing(GearBox.fromReductionStages(50)))
   .withMotorInverted(false)
   .withIdleMode(MotorMode.BRAKE)
   .withSoftLimit(Degrees.of(0), Degrees.of(90))
   .withStatorCurrentLimit(Amps.of(40))
   .withClosedLoopRampRate(Seconds.of(0.1))
  .withOpenLoopRampRate(Seconds.of(0.1));

  private SmartMotorController smc = new SparkWrapper(hoodSpark, DCMotor.getNEO(1),
   smcConfig);

  private PivotConfig hoodConfig = new PivotConfig(smc)
  .withHardLimit(Degrees.of(-5), Degrees.of(95))
  .withStartingPosition(Degrees.of(0))
  .withMOI(0.001)
  .withTelemetry("Hood", TelemetryVerbosity.HIGH);

   private yams.mechanisms.positional.Pivot hood = new Pivot(hoodConfig);

  public Hood() {
  }

   public Command setDegreeCommand(double degree) {
    return hood.setAngle(Degrees.of(degree));
  }

  public void setAngleSetpoint(Angle degree) {
      hood.setMechanismPositionSetpoint(degree);
  }

  public Angle getAngle() {
    return hood.getAngle();
  }

  public void setDutyCycleSetpoint(double dutyCycle) {
    hood.setDutyCycleSetpoint(dutyCycle);
  } 

  public Command setDutyCycle(double dutyCycle) {
    return hood.set(dutyCycle);
  }

  public Command stopCommand(){
    return hood.set(0);
  }
  @Override
  public void periodic() {
    hood.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    hood.simIterate();
  }

}