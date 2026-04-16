package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Landmarks;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

public class PrepareShotCommand extends Command {
    private static final InterpolatingTreeMap<Distance, Shot> distanceToShotMap = new InterpolatingTreeMap<>(
        (startValue, endValue, q) -> 
            InverseInterpolator.forDouble()
                .inverseInterpolate(startValue.in(Meters), endValue.in(Meters), q.in(Meters)),
        (startValue, endValue, t) ->
            new Shot(
                Interpolator.forDouble()
                    .interpolate(startValue.shooterRPM, endValue.shooterRPM, t),
                Interpolator.forDouble()
                    .interpolate(startValue.hoodPosition, endValue.hoodPosition, t)
            )
    );

    static {
        distanceToShotMap.put(Inches.of(10.5), new Shot(2900, 0.27));
        distanceToShotMap.put(Inches.of(33.0), new Shot(3100, 0.35));
        distanceToShotMap.put(Inches.of(41), new Shot(3000, 0.40));
        distanceToShotMap.put(Inches.of(71), new Shot(3500, 0.45));
        distanceToShotMap.put(Inches.of(90.5), new Shot(3600, 0.55));
        distanceToShotMap.put(Inches.of(115), new Shot(3750, 0.65));
        distanceToShotMap.put(Inches.of(128), new Shot(3800, 0.77));
        distanceToShotMap.put(Inches.of(131.5), new Shot(3850, 0.77));
        distanceToShotMap.put(Inches.of(142.0), new Shot(4050, 0.77));
        distanceToShotMap.put(Inches.of(152.0), new Shot(4150, 0.77));
    }

    private final Shooter shooter;
    private final Hood hood;
    private final Supplier<Pose2d> robotPoseSupplier;

    public PrepareShotCommand(Shooter shooter, Hood hood, Supplier<Pose2d> robotPoseSupplier) {
        this.shooter = shooter;
        this.hood = hood;
        this.robotPoseSupplier = robotPoseSupplier;
        addRequirements(shooter, hood);
    }

    public boolean isReadyToShoot() {
        return shooter.isVelocityWithinTolerance() && hood.isPositionWithinTolerance();
    }

    private Distance getDistanceToHub() {
        final Translation2d robotPosition = robotPoseSupplier.get().getTranslation();
        final Translation2d hubPosition = Landmarks.hubPosition();
        return Meters.of(robotPosition.getDistance(hubPosition));
    }

    private Distance getDistanceToDepot() {
        final Translation2d robotPosition = robotPoseSupplier.get().getTranslation();
        final Translation2d hubPosition = Landmarks.depotPosition();
        return Meters.of(robotPosition.getDistance(hubPosition));
    }

    private Distance getDistanceToOutpost() {
        final Translation2d robotPosition = robotPoseSupplier.get().getTranslation();
        final Translation2d hubPosition = Landmarks.outpostPosition();
        return Meters.of(robotPosition.getDistance(hubPosition));
    }

    @Override
    public void execute() {
        final Distance distanceToTarget;
        if (RobotContainer.mech.a().getAsBoolean()){
            if (getDistanceToDepot().in(Inches) < getDistanceToOutpost().in(Inches)) {
                distanceToTarget = getDistanceToDepot();
            }
            else {
                distanceToTarget = getDistanceToOutpost();
            }
        } 
        else {
            distanceToTarget = getDistanceToHub();
        }
        
        final Shot shot = distanceToShotMap.get(distanceToTarget);
        shooter.setRPM(shot.shooterRPM);
        shooter.setDashboardTargetRPM(shot.shooterRPM);
        hood.setPosition(shot.hoodPosition);
        SmartDashboard.putNumber("Distance to Hub (inches)", distanceToTarget.in(Inches));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

    public static class Shot {
        public final double shooterRPM;
        public final double hoodPosition;

        public Shot(double shooterRPM, double hoodPosition) {
            this.shooterRPM = shooterRPM;
            this.hoodPosition = hoodPosition;
        }
    }
}
