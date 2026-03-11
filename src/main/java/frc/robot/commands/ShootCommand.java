package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;

import java.util.List;
import java.util.Optional;

import frc.robot.subsystems.*;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;


public class ShootCommand extends Command {

    private record RecordedShot(Distance distance, AngularVelocity shooterSpeed, Angle hoodAngle, Time tof) {
        public Pair<Double, Double> getRPM() {
            return Pair.of(distance.in(Meters), shooterSpeed.in(RPM));
        }

        public Pair<Double, Double> getTOF() {
            return Pair.of(distance.in(Meters), tof.in(Second));
        }

        public Pair<Double, Double> getHoodAngle() {
            return Pair.of(distance.in(Meters), hoodAngle.in(Degrees));
        }
    }

    private final Shooter shooter;
    private final Kicker kicker;
    private final Optional<SwerveSubsystem> swerve;
    private final Hood hood;//94 rpm
    private final AngularVelocity goalRPM;   // <-- parameter stored here
    private final Angle goalDegree;

    private final Debouncer shootDebounce1 = new Debouncer(0.3, DebounceType.kFalling);

    private final List<RecordedShot> shots = List.of(
            // TUNE HERE
            new RecordedShot(Meters.of(1), RPM.of(1000), Degrees.of(0), Second.of(1)),
            new RecordedShot(Meters.of(2), RPM.of(2000), Degrees.of(0),Second.of(1)),
            new RecordedShot(Meters.of(3), RPM.of(3000), Degrees.of(0),Second.of(1))

    );
    private final InterpolatingDoubleTreeMap calculatedGoalRPM = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap calculatedTOF = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap calculatedHoodAngle = new InterpolatingDoubleTreeMap();

    public ShootCommand(
            Shooter shooter,
            Kicker kicker,
            Hood hood,
            AngularVelocity goalRPM1,
            Angle goalDegree1   // <-- parameter passed in
    ) {
        this.shooter = shooter;
        this.kicker = kicker;
        this.hood = hood;
        this.swerve = Optional.empty();
        
        this.goalRPM = goalRPM1;   // <-- store parameter
        this.goalDegree = goalDegree1;

        addRequirements(this.shooter, this.kicker, this.hood);
    }

    public ShootCommand(
            Shooter shooter,
            Kicker kicker,
            Hood hood,
            SwerveSubsystem swerve) {
        this.shooter = shooter;
        this.kicker = kicker;
        this.hood = hood;
        this.swerve = Optional.of(swerve);
        goalRPM = RPM.zero();
        goalDegree = Degrees.zero();

        for (var shot : shots) {
            calculatedGoalRPM.put(shot.distance.in(Meters), shot.shooterSpeed.in(RPM));
            calculatedTOF.put(shot.distance.in(Meters), shot.tof.in(Second));
            calculatedHoodAngle.put(shot.distance.in(Meters), shot.hoodAngle.in(Degrees));
        }
        addRequirements(this.shooter, this.kicker, this.hood);
    }

    

    @Override
    public void initialize() {
        // Spin up shooter to the passed RPM
        shooter.setVelocitySetpoint(goalRPM);
        if (swerve.isEmpty()) {
            hood.setAngleSetpoint(goalDegree);
        }
        
        
    }

    @Override
    public void execute() {

        AngularVelocity goalRPM1 = goalRPM;
        Angle goalDegree1 = goalDegree;
        if (swerve.isPresent()) {
            goalRPM1 = RPM.of(calculatedGoalRPM.get(swerve.get().distanceToHub()));
            goalDegree1 = Degrees.of(calculatedHoodAngle.get(swerve.get().distanceToHub()));
        }

        shooter.setVelocitySetpoint(goalRPM1);
        hood.setAngleSetpoint(goalDegree1);

        AngularVelocity shooterRPM = shooter.getRPM();

        boolean shooterReady = shootDebounce1.calculate(
                shooterRPM.isNear(
                        goalRPM1,
                        RPM.of(100)// tolerance
                )

        );

        if (shooterReady) {
            kicker.feedCommand();
  
        } else {
            kicker.stopCommand();
        }


    }

    @Override
    public void end(boolean interrupted) {
        shooter.setDutyCycleSetpoint(0);
        kicker.stopCommand();
        hood.setDutyCycleSetpoint(0);

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}