// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Superstructure extends SubsystemBase {
  /** Creates a new Superstructure. */
   public final Hood hood;
   public final Intake intake;
   public final Kicker kicker;
   private final Pivot pivot;
   public final Shooter shooter;

  private static final AngularVelocity SHOOTER_TOLERANCE = RPM.of(100);
  private static final Angle HOOD_TOLERANCE = Degrees.of(2);
    

  private final Trigger isShooterAtSpeed;
  private final Trigger isHoodOnTarget;
  private final Trigger isReadyToShoot;

  private AngularVelocity targetShooterSpeed = RPM.of(0);
  private Angle targetHoodAngle = Degrees.of(0);

  public Superstructure(Hood hood, Intake intake, Kicker kicker, Pivot pivot, Shooter shooter) {
    this.hood = hood;
    this.intake = intake;
    this.kicker = kicker;
    this.pivot = pivot;
    this.shooter = shooter;

    this.isShooterAtSpeed = new Trigger(
        () -> Math.abs(shooter.getSpeed().in(RPM) - targetShooterSpeed.in(RPM)) < SHOOTER_TOLERANCE.in(RPM));

        
    this.isHoodOnTarget = new Trigger(
        () -> Math.abs(hood.getAngle().in(Degrees) - targetHoodAngle.in(Degrees)) < HOOD_TOLERANCE.in(Degrees));

    this.isReadyToShoot = isShooterAtSpeed.and(isHoodOnTarget);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
