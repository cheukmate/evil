// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Power;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeWheels extends SubsystemBase {
  /** Creates a new IntakeWheels. */
  SparkFlex wheels  = new SparkFlex(17, MotorType.kBrushless);

  public IntakeWheels() {}

  public void Intake(double speed){
    wheels.set(speed);
  }

  

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
