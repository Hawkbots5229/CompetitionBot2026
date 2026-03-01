package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Ports {
    // CAN Buses
    public static final CANBus kRoboRioCANBus = new CANBus("rio");
    public static final CANBus kCANivoreCANBus = new CANBus("main");

    // Talon FX IDs
    public static final int kIntakePivot = 11;
    public static final int kIntakeRollers = 12;
    public static final int kFloor = 13;
    public static final int kFeeder = 14;
    public static final int kShooterLeft = 15;
    public static final int kShooterMiddle = 16;
    public static final int kShooterRight = 17;
    public static final int kHanger = 18;

    // PWM Ports
    public static final int kHoodLeftServo = 3;
    public static final int kHoodRightServo = 4;
}
