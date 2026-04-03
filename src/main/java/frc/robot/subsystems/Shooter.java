package frc.robot.subsystems;






import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

import yams.motorcontrollers.remote.TalonFXWrapper;

public class Shooter extends SubsystemBase {
  // 2 krakens, 4in shooter wheels
  
  private final TalonFX leaderTalon = new TalonFX(Constants.IDConstants.FLYWHEEL_MOTOR_MAIN_KRAKEN);

  private final TalonFX followerTalon = new TalonFX(Constants.IDConstants.FLYWHEEL_MOTOR_FOLLOWER_KRAKEN);

  private final SmartMotorControllerConfig shooterSmartMotorControllerConfig = new SmartMotorControllerConfig(this)
      .withFollowers(Pair.of(followerTalon, false)) //lowkirkenuinely dont know what the second parameter does ngl twin
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(0.00036, 0, 0) // Change twin! .0000036 or smth idk
      .withFeedforward(new SimpleMotorFeedforward(0.191, 0.11858, 0.0)) // 0.191, 0.11858, 0.0 
      .withTelemetry("ShooterMotor", TelemetryVerbosity.LOW)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(50));

  private final SmartMotorController shooterSmartMotorController = new TalonFXWrapper(leaderTalon, DCMotor.getKrakenX60(2), shooterSmartMotorControllerConfig);

  private final FlyWheelConfig shooterConfig = new FlyWheelConfig(shooterSmartMotorController)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(1))
      .withUpperSoftLimit(RPM.of(6000))
      .withLowerSoftLimit(RPM.of(0))
      .withTelemetry("Shooter", TelemetryVerbosity.LOW);

  private final FlyWheel shooter = new FlyWheel(shooterConfig);

  public Shooter() {
    
  }

 public AngularVelocity getRPM() {
        return shooter.getSpeed();
    }

    public Command setVelocityCommand(AngularVelocity velocity) {
        return shooter.setSpeed(velocity);
    }

    public void setVelocitySetpoint(AngularVelocity velocity)
    {
        shooter.setMechanismVelocitySetpoint(velocity);
    }

    public Command setDutyCycle(double dutyCycle) {
        return shooter.set(dutyCycle);
    }

    public Command stopCommand() {
        return shooter.set(0);
    }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/LeaderVelocity", leaderTalon.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/FollowerVelocity", followerTalon.getVelocity().getValueAsDouble());


    SmartDashboard.putNumber("Shooter Both Velocity", shooter.getSpeed().baseUnitMagnitude());
    
  }

  @Override
  public void simulationPeriodic() {
   shooter.simIterate();
  }

  private Distance wheelRadius() {
    return Inches.of(4).div(2);
  }

    public void setDutyCycleSetpoint(double dutyCycle) {
         shooter.setDutyCycleSetpoint(0);
    }

  public LinearVelocity getTangentialVelocity() {
    // Calculate tangential velocity at the edge of the wheel and convert to
    // LinearVelocity

    return MetersPerSecond.of(getRPM().in(RadiansPerSecond)
        * wheelRadius().in(Meters));
  }
}