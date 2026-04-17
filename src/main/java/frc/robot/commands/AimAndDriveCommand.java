package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Driving;
import frc.robot.Landmarks;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;
import frc.util.DriveInputSmoother;
import frc.util.GeometryUtil;
import frc.util.ManualDriveInput;

public class AimAndDriveCommand extends Command {
    private static final Angle kAimTolerance = Degrees.of(5);

    private final Swerve swerve;
    private final DriveInputSmoother inputSmoother;

    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleRequest = new SwerveRequest.FieldCentricFacingAngle()
        .withRotationalDeadband(Driving.kPIDRotationDeadband)
        .withMaxAbsRotationalRate(Driving.kMaxRotationalRate)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
        .withHeadingPID(5.0, 0, 0.5);

    public AimAndDriveCommand(
        Swerve swerve,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput
    ) {
        this.swerve = swerve;
        this.inputSmoother = new DriveInputSmoother(forwardInput, leftInput);
        addRequirements(swerve);
    }

    public AimAndDriveCommand(Swerve swerve) {
        this(swerve, () -> 0, () -> 0);
    }

    public boolean isAimed() {
        final Rotation2d targetHeading = fieldCentricFacingAngleRequest.TargetDirection;
        final Rotation2d currentHeadingInBlueAlliancePerspective = swerve.getState().Pose.getRotation();
        final Rotation2d currentHeadingInOperatorPerspective = currentHeadingInBlueAlliancePerspective.rotateBy(swerve.getOperatorForwardDirection());
        return GeometryUtil.isNear(targetHeading, currentHeadingInOperatorPerspective, kAimTolerance);
    }

    private Rotation2d getDirectionToHub() {
        final Translation2d hubPosition = Landmarks.hubPosition();
        final Translation2d robotPosition = swerve.getState().Pose.getTranslation();
        final Rotation2d hubDirectionInBlueAlliancePerspective = hubPosition.minus(robotPosition).getAngle();
        final Rotation2d hubDirectionInOperatorPerspective = hubDirectionInBlueAlliancePerspective.rotateBy(swerve.getOperatorForwardDirection());
        return hubDirectionInOperatorPerspective;
    }

    private Rotation2d getDirectionToDepot() {
        final Translation2d depotPosition = Landmarks.depotPosition();
        final Translation2d robotPosition = swerve.getState().Pose.getTranslation();
        final Rotation2d depotDirectionInBlueAlliancePerspective = depotPosition.minus(robotPosition).getAngle();
        final Rotation2d depotDirectionInOperatorPerspective = depotDirectionInBlueAlliancePerspective.rotateBy(swerve.getOperatorForwardDirection());
        return depotDirectionInOperatorPerspective;
    }

    private Rotation2d getDirectionToOutpost() {
        final Translation2d outpostPosition = Landmarks.outpostPosition();
        final Translation2d robotPosition = swerve.getState().Pose.getTranslation();
        final Rotation2d outpostDirectionInBlueAlliancePerspective = outpostPosition.minus(robotPosition).getAngle();
        final Rotation2d outpostDirectionInOperatorPerspective = outpostDirectionInBlueAlliancePerspective.rotateBy(swerve.getOperatorForwardDirection());
        return outpostDirectionInOperatorPerspective;
    }

    private Distance getDistanceToDepot() {
        final Translation2d robotPosition = swerve.getState().Pose.getTranslation();
        final Translation2d hubPosition = Landmarks.depotPosition();
        return Meters.of(robotPosition.getDistance(hubPosition));
    }

    private Distance getDistanceToOutpost() {
        final Translation2d robotPosition = swerve.getState().Pose.getTranslation();
        final Translation2d hubPosition = Landmarks.outpostPosition();
        return Meters.of(robotPosition.getDistance(hubPosition));
    }

    @Override
    public void execute() {
        final ManualDriveInput input = inputSmoother.getSmoothedInput();
        final Rotation2d targetDirection;

        if (RobotContainer.mech.a().getAsBoolean()){
            if (getDistanceToDepot().in(Inches) < getDistanceToOutpost().in(Inches)) {
                targetDirection = getDirectionToDepot();
            }
            else {
                targetDirection = getDirectionToOutpost();
            }
        } 
        else {
            targetDirection = getDirectionToHub();
        }
        if (isAimed() && !RobotContainer.mech.a().getAsBoolean()) {
            swerve.xLockCommand();
        }
        else {
            swerve.setControl(
                fieldCentricFacingAngleRequest
                    .withVelocityX(Driving.kMaxSpeed.times(input.forward))
                    .withVelocityY(Driving.kMaxSpeed.times(input.left))
                    .withTargetDirection(targetDirection)
            );
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
