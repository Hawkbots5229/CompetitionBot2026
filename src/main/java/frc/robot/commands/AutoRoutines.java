// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.generated.ChoreoTraj.OutpostTrajectory$0;
import static frc.robot.generated.ChoreoTraj.OutpostTrajectory$1;
import static frc.robot.generated.ChoreoTraj.MiddleTrajectory;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public final class AutoRoutines {
    private final Swerve swerve;
    private final Intake intake;
    private final Floor floor;
    private final Feeder feeder;
    private final Shooter shooter;
    private final Hood hood;
    private final Hanger hanger;
    private final Limelight limelight;

    private final SubsystemCommands subsystemCommands;

    private final AutoFactory autoFactory;
    private final AutoChooser autoChooser;

    public AutoRoutines(
        Swerve swerve,
        Intake intake,
        Floor floor,
        Feeder feeder,
        Shooter shooter,
        Hood hood,
        Hanger hanger,
        Limelight limelight
    ) {
        this.swerve = swerve;
        this.intake = intake;
        this.floor = floor;
        this.feeder = feeder;
        this.shooter = shooter;
        this.hood = hood;
        this.hanger = hanger;
        this.limelight = limelight;

        this.subsystemCommands = new SubsystemCommands(swerve, intake, floor, feeder, shooter, hood, hanger);

        this.autoFactory = swerve.createAutoFactory();
        this.autoChooser = new AutoChooser();
    }

    public void configure() {
        autoChooser.addRoutine("Outpost", this::outpostRoutine);
        autoChooser.addRoutine("Middle", this::middleRoutine);
        SmartDashboard.putData("Auto Chooser", autoChooser);
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

    private AutoRoutine outpostRoutine() {
        final AutoRoutine routine = autoFactory.newRoutine("Outpost");
        final AutoTrajectory startToOutpostPose = OutpostTrajectory$0.asAutoTraj(routine);
        final AutoTrajectory outpostToShootingPose = OutpostTrajectory$1.asAutoTraj(routine);
        final PrepareShotCommand prepareShotCommand = new PrepareShotCommand(shooter, hood, () -> swerve.getState().Pose);

        routine.active().onTrue(
            Commands.sequence(
                startToOutpostPose.resetOdometry(),
                startToOutpostPose.cmd()
            )
        );

        startToOutpostPose.active().whileTrue(limelight.idle());

        startToOutpostPose.done().onTrue(
            Commands.waitUntil(hanger::isHomed).andThen(
                Commands.sequence(
                    Commands.waitSeconds(1.0),
                    intake.runOnce(() -> intake.set(Intake.Position.INTAKE)),
                    outpostToShootingPose.cmd())
            )
        );

        outpostToShootingPose.active().whileTrue(limelight.idle());
        outpostToShootingPose.atTimeBeforeEnd(1.0).onTrue(intake.intakeCommand());

        outpostToShootingPose.done().onTrue(
            Commands.waitUntil(hanger::isHomed).andThen(
                Commands.sequence(
                    Commands.waitSeconds(0.5),
                    intake.runOnce(() -> intake.set(Intake.Position.INTAKE)),
                    Commands.parallel(
                        prepareShotCommand,
                        Commands.waitUntil(prepareShotCommand::isReadyToShoot)
                            .andThen(feed())
                    )
                )
            )
        );
        
        return routine;
    }

    private AutoRoutine middleRoutine() {
        final AutoRoutine routine = autoFactory.newRoutine("Middle");
        final AutoTrajectory startToShootingPose = MiddleTrajectory.asAutoTraj(routine);
        final PrepareShotCommand prepareShotCommand = new PrepareShotCommand(shooter, hood, () -> swerve.getState().Pose);

        routine.active().onTrue(
            Commands.sequence(
                startToShootingPose.resetOdometry(),
                startToShootingPose.cmd()
            )    
        );
        
        startToShootingPose.active().whileTrue(limelight.idle());
        
        startToShootingPose.done().onTrue(
            Commands.waitUntil(hanger::isHomed).andThen(
                Commands.sequence(
                    Commands.waitSeconds(0.5),
                    intake.runOnce(() -> intake.set(Intake.Position.INTAKE)),
                    Commands.parallel(
                        prepareShotCommand,
                        Commands.waitUntil(prepareShotCommand::isReadyToShoot)
                            .andThen(feed())
                    )
                )
            )
        );        
        
        return routine;
    }

    private Command feed() {
        return 
            Commands.parallel(
                feeder.feedCommand(),
                Commands.waitSeconds(0.125)
                    .andThen(floor.feedCommand().alongWith(intake.agitateCommand()))
            );
    }
}
