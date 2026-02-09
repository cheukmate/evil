// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter2PleaseWorkPLease extends SubsystemBase {
  /** Creates a new Shooter2PleaseWorkPLease. */

  TalonFX shootermotor1 = new TalonFX(12);
  TalonFX shootermotor2 = new TalonFX(13);

  final DutyCycleOut m_1request = new DutyCycleOut(0.0);
  final DutyCycleOut m_2request = new DutyCycleOut(0.0);

  public Shooter2PleaseWorkPLease() {
shootermotor1.setControl(new Follower(shootermotor2.getDeviceID(), MotorAlignmentValue.Aligned));


  }

  public void Runmotor(){
  shootermotor1.setControl(m_1request.withOutput(-.6));
  shootermotor2.setControl(m_2request.withOutput(.6));


  }

  public void stopMotor(){
  shootermotor1.setControl(m_1request.withOutput(0));
  shootermotor2.setControl(m_2request.withOutput(0));


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
