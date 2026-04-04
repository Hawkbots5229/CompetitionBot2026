package frc.util;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;

public class DriveInputSmoother {
    private static final double kJoystickDeadband = 0.15;
    private static final double kCurveExponent = 1.5;

    private final DoubleSupplier forwardInput;
    private final DoubleSupplier leftInput;
    private final DoubleSupplier rotationInput;

    public DriveInputSmoother(DoubleSupplier forwardInput, DoubleSupplier leftInput, DoubleSupplier rotationInput) {
        this.forwardInput = forwardInput;
        this.leftInput = leftInput;
        this.rotationInput = rotationInput;
    }

    public DriveInputSmoother(DoubleSupplier forwardInput, DoubleSupplier leftInput) {
        this(forwardInput, leftInput, () -> 0);
    }

    public ManualDriveInput getSmoothedInput() { 
        double driveScale = Math.max(0.3, 1.0-(RobotContainer.driver.getLeftTriggerAxis()*0.4)-(RobotContainer.driver.getRightTriggerAxis()*0.4));
        SmartDashboard.putNumber("DriveScale", driveScale);
        final Vector<N2> rawTranslationInput = VecBuilder.fill(forwardInput.getAsDouble()*driveScale, leftInput.getAsDouble()*driveScale);
        final Vector<N2> deadbandedTranslationInput = MathUtil.applyDeadband(rawTranslationInput, kJoystickDeadband);
        final Vector<N2> curvedTranslationInput = MathUtil.copyDirectionPow(deadbandedTranslationInput, kCurveExponent);

        final double rawRotationInput = rotationInput.getAsDouble()*driveScale;
        final double deadbandedRotationInput = MathUtil.applyDeadband(rawRotationInput, kJoystickDeadband);
        final double curvedRotationInput = MathUtil.copyDirectionPow(deadbandedRotationInput, kCurveExponent);

        return new ManualDriveInput(
            curvedTranslationInput.get(0), 
            curvedTranslationInput.get(1), 
            curvedRotationInput
        );
    }
}
