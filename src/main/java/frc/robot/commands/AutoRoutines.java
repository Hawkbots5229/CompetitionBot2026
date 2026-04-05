// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.generated.ChoreoTraj.ChaosLeftShootTraj;
import static frc.robot.generated.ChoreoTraj.ChaosRightShootTraj;
import static frc.robot.generated.ChoreoTraj.ClimbShootTraj$0;
import static frc.robot.generated.ChoreoTraj.ClimbShootTraj$1;
import static frc.robot.generated.ChoreoTraj.ClimbShootTraj$2;
import static frc.robot.generated.ChoreoTraj.DepotShootTraj$0;
import static frc.robot.generated.ChoreoTraj.DepotShootTraj$1;
import static frc.robot.generated.ChoreoTraj.DepotShootTraj$2;
import static frc.robot.generated.ChoreoTraj.DepotShootTraj$3;
import static frc.robot.generated.ChoreoTraj.DepotShootTraj$4;
import static frc.robot.generated.ChoreoTraj.FieldLeftShootTraj$0;
import static frc.robot.generated.ChoreoTraj.FieldLeftShootTraj$1;
import static frc.robot.generated.ChoreoTraj.FieldLeftShootTraj$2;
import static frc.robot.generated.ChoreoTraj.FieldRightShootTraj$0;
import static frc.robot.generated.ChoreoTraj.FieldRightShootTraj$1;
import static frc.robot.generated.ChoreoTraj.FieldRightShootTraj$2;
import static frc.robot.generated.ChoreoTraj.HubCenterShootTraj;
import static frc.robot.generated.ChoreoTraj.HubLeftShootTraj;
import static frc.robot.generated.ChoreoTraj.HubRightShootTraj;
import static frc.robot.generated.ChoreoTraj.OutpostShootTraj$0;
import static frc.robot.generated.ChoreoTraj.OutpostShootTraj$1;
import static frc.robot.generated.ChoreoTraj.OutpostShootTraj$2;
import static frc.robot.generated.ChoreoTraj.OutpostShootTraj$3;
import static frc.robot.generated.ChoreoTraj.OutpostShootTraj$4;

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
        autoChooser.addRoutine("ChaosLeftShoot", this::ChaosLeftShootRoutine);
        autoChooser.addRoutine("ChaosRightShoot", this::ChaosRightShootRoutine);
        autoChooser.addRoutine("ClimbShoot", this::ClimbShootRoutine);
        autoChooser.addRoutine("DepotShoot", this::DepotShootRoutine);
        autoChooser.addRoutine("DepotShootClimb", this::DepotShootClimbRoutine);
        autoChooser.addRoutine("FieldLeftShootRoutine", this::FieldLeftShootRoutine);
        autoChooser.addRoutine("FieldRightShootRoutine", this::FieldRightShootRoutine);
        autoChooser.addRoutine("HubCenterShoot", this::HubCenterShootRoutine);
        autoChooser.addRoutine("HubLeftShoot", this::HubLeftShootRoutine);
        autoChooser.addRoutine("HubRightShoot", this::HubRightShootRoutine);
        autoChooser.addRoutine("OutpostShoot", this::OutpostShootRoutine);
        autoChooser.addRoutine("OutpostShootClimb", this::OutpostShootClimbRoutine);
        SmartDashboard.putData("Auto Chooser", autoChooser);
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }


    private AutoRoutine ChaosLeftShootRoutine() {
        final AutoRoutine routine = autoFactory.newRoutine("ChaosLeftShoot");
        final AutoTrajectory startToShootingPose = ChaosLeftShootTraj.asAutoTraj(routine);
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

    private AutoRoutine ChaosRightShootRoutine() {
        final AutoRoutine routine = autoFactory.newRoutine("ChaosRightShoot");
        final AutoTrajectory startToShootingPose = ChaosRightShootTraj.asAutoTraj(routine);
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


    private AutoRoutine ClimbShootRoutine() {
        final AutoRoutine routine = autoFactory.newRoutine("ClimbShoot");
        final AutoTrajectory startToShootingPose = ClimbShootTraj$0.asAutoTraj(routine);
        final AutoTrajectory shootingPoseToApproachPose = ClimbShootTraj$1.asAutoTraj(routine);
        final AutoTrajectory approachPoseToClimbPose = ClimbShootTraj$2.asAutoTraj(routine);
        final PrepareShotCommand prepareShotCommand = new PrepareShotCommand(shooter, hood, () -> swerve.getState().Pose);

        routine.active().onTrue(
            Commands.sequence(
                startToShootingPose.resetOdometry(),
                startToShootingPose.cmd()
            )    
        );
        
        startToShootingPose.active().whileTrue(limelight.idle()); 
        shootingPoseToApproachPose.active().whileTrue(limelight.idle()); 
        approachPoseToClimbPose.active().whileTrue(limelight.idle());  
        
        startToShootingPose.done().onTrue(
            Commands.waitUntil(hanger::isHomed).andThen(
                Commands.sequence(
                    Commands.waitSeconds(0.5),
                    intake.runOnce(() -> intake.set(Intake.Position.INTAKE)),
                    Commands.parallel(
                        prepareShotCommand,
                        Commands.waitUntil(prepareShotCommand::isReadyToShoot)
                            .andThen(feed())
                    ).withTimeout(4.0),
                    intake.runOnce(() -> intake.set(Intake.Position.STOWED)),
                    shootingPoseToApproachPose.cmd()
                )              
            )
        );

        shootingPoseToApproachPose.done().onTrue(
                Commands.sequence(
                    hanger.positionCommand(Hanger.Position.HANGING),
                    Commands.waitSeconds(0.25),
                    approachPoseToClimbPose.cmd()
                )
        );

        approachPoseToClimbPose.done().onTrue(
            hanger.positionCommand(Hanger.Position.HUNG)
        );
        
        return routine;
    }


    private AutoRoutine DepotShootRoutine() {
        final AutoRoutine routine = autoFactory.newRoutine("DepotShoot");
        final AutoTrajectory startToApproachPose = DepotShootTraj$0.asAutoTraj(routine);
        final AutoTrajectory approachPoseToCollectPose = DepotShootTraj$1.asAutoTraj(routine);
        final AutoTrajectory collectPoseToShootingPose = DepotShootTraj$2.asAutoTraj(routine);
        final PrepareShotCommand prepareShotCommand = new PrepareShotCommand(shooter, hood, () -> swerve.getState().Pose);

        routine.active().onTrue(
            Commands.sequence(
                startToApproachPose.resetOdometry(),
                startToApproachPose.cmd()
            )    
        );
        
        startToApproachPose.active().whileTrue(limelight.idle()); 
        approachPoseToCollectPose.active().whileTrue(limelight.idle()); 
        collectPoseToShootingPose.active().whileTrue(limelight.idle());  
        
        startToApproachPose.done().onTrue(
            Commands.waitUntil(hanger::isHomed).andThen(
                Commands.sequence(
                    Commands.waitSeconds(0.5),
                    intake.runOnce(() -> intake.set(Intake.Position.INTAKE)),
                    approachPoseToCollectPose.cmd()
                )              
            )
        );

        approachPoseToCollectPose.done().onTrue(
                Commands.sequence(
                    Commands.waitSeconds(5.0),
                    collectPoseToShootingPose.cmd()
                )
        );

        collectPoseToShootingPose.done().onTrue(
            Commands.parallel(
                prepareShotCommand,
                Commands.waitUntil(prepareShotCommand::isReadyToShoot)
                    .andThen(feed())
            )
        );
        
        return routine;
    }


    private AutoRoutine DepotShootClimbRoutine() {
        final AutoRoutine routine = autoFactory.newRoutine("DepotShootClimb");
        final AutoTrajectory startToApproachPose = DepotShootTraj$0.asAutoTraj(routine);
        final AutoTrajectory approachPoseToCollectPose = DepotShootTraj$1.asAutoTraj(routine);
        final AutoTrajectory collectPoseToShootingPose = DepotShootTraj$2.asAutoTraj(routine);
        final AutoTrajectory shootingPoseToApproachPose = DepotShootTraj$3.asAutoTraj(routine);
        final AutoTrajectory approachPoseToClimbPose = DepotShootTraj$4.asAutoTraj(routine);
        final PrepareShotCommand prepareShotCommand = new PrepareShotCommand(shooter, hood, () -> swerve.getState().Pose);

        routine.active().onTrue(
            Commands.sequence(
                startToApproachPose.resetOdometry(),
                startToApproachPose.cmd()
            )    
        );
        
        startToApproachPose.active().whileTrue(limelight.idle()); 
        approachPoseToCollectPose.active().whileTrue(limelight.idle()); 
        collectPoseToShootingPose.active().whileTrue(limelight.idle());  
        shootingPoseToApproachPose.active().whileTrue(limelight.idle());
        approachPoseToClimbPose.active().whileTrue(limelight.idle());
        
        startToApproachPose.done().onTrue(
            Commands.waitUntil(hanger::isHomed).andThen(
                Commands.sequence(
                    Commands.waitSeconds(0.5),
                    intake.runOnce(() -> intake.set(Intake.Position.INTAKE)),
                    approachPoseToCollectPose.cmd()
                )              
            )
        );

        approachPoseToCollectPose.done().onTrue(
                Commands.sequence(
                    Commands.waitSeconds(5.0),
                    collectPoseToShootingPose.cmd()
                )
        );

        collectPoseToShootingPose.done().onTrue(
            Commands.sequence(
                Commands.parallel(
                    prepareShotCommand,
                    Commands.waitUntil(prepareShotCommand::isReadyToShoot)
                        .andThen(feed())
                ).withTimeout(4.0),
                intake.runOnce(() -> intake.set(Intake.Position.STOWED)),
                shootingPoseToApproachPose.cmd()
            )
        );

        shootingPoseToApproachPose.done().onTrue(
            Commands.sequence(
                hanger.positionCommand(Hanger.Position.HANGING),
                Commands.waitSeconds(0.25),
                approachPoseToClimbPose.cmd()
            )
        );

        approachPoseToClimbPose.done().onTrue(
            hanger.positionCommand(Hanger.Position.HUNG)
        );
        
        return routine;
    }


    private AutoRoutine FieldLeftShootRoutine() {
        final AutoRoutine routine = autoFactory.newRoutine("FieldLeftShoot");
        final AutoTrajectory startToFieldPose = FieldLeftShootTraj$0.asAutoTraj(routine);
        final AutoTrajectory fieldPoseToCollectPose = FieldLeftShootTraj$1.asAutoTraj(routine);
        final AutoTrajectory collectPoseToShootingPose = FieldLeftShootTraj$2.asAutoTraj(routine);
        final PrepareShotCommand prepareShotCommand = new PrepareShotCommand(shooter, hood, () -> swerve.getState().Pose);

        routine.active().onTrue(
                Commands.sequence(
                    startToFieldPose.resetOdometry(),
                    startToFieldPose.cmd()
                ) 
        );

        startToFieldPose.active().whileTrue(limelight.idle());
        fieldPoseToCollectPose.active().whileTrue(limelight.idle());
        collectPoseToShootingPose.active().whileTrue(limelight.idle());

        startToFieldPose.done().onTrue(
            Commands.waitUntil(hanger::isHomed).andThen(
                Commands.sequence(
                    Commands.waitSeconds(0.5),
                    intake.runOnce(() -> intake.set(Intake.Position.INTAKE)),

                    Commands.deadline(
                        fieldPoseToCollectPose.cmd(),
                        Commands.startEnd(
                            () -> intake.set(Intake.Speed.INTAKE), 
                            () -> intake.set(Intake.Speed.STOP),
                            intake
                        )
                    )
                )
            )
        );

        fieldPoseToCollectPose.done().onTrue(
            collectPoseToShootingPose.cmd()
        );

        collectPoseToShootingPose.done().onTrue(
            Commands.parallel(
                prepareShotCommand,
                Commands.waitUntil(prepareShotCommand::isReadyToShoot)
                    .andThen(feed())
            )
        );
        
        return routine;
    }


    private AutoRoutine FieldRightShootRoutine() {
        final AutoRoutine routine = autoFactory.newRoutine("FieldRightShoot");
        final AutoTrajectory startToFieldPose = FieldRightShootTraj$0.asAutoTraj(routine);
        final AutoTrajectory fieldPoseToCollectPose = FieldRightShootTraj$1.asAutoTraj(routine);
        final AutoTrajectory collectPoseToShootingPose = FieldRightShootTraj$2.asAutoTraj(routine);
        final PrepareShotCommand prepareShotCommand = new PrepareShotCommand(shooter, hood, () -> swerve.getState().Pose);

        routine.active().onTrue(
                Commands.sequence(
                    startToFieldPose.resetOdometry(),
                    startToFieldPose.cmd()
                ) 
        );

        startToFieldPose.active().whileTrue(limelight.idle());
        fieldPoseToCollectPose.active().whileTrue(limelight.idle());
        collectPoseToShootingPose.active().whileTrue(limelight.idle());

        startToFieldPose.done().onTrue(
            Commands.waitUntil(hanger::isHomed).andThen(
                Commands.sequence(
                    Commands.waitSeconds(0.5),
                    intake.runOnce(() -> intake.set(Intake.Position.INTAKE)),

                    Commands.deadline(
                        fieldPoseToCollectPose.cmd(),
                        Commands.startEnd(
                            () -> intake.set(Intake.Speed.INTAKE), 
                            () -> intake.set(Intake.Speed.STOP),
                            intake
                        )
                    )
                )
            )
        );

        fieldPoseToCollectPose.done().onTrue(
            collectPoseToShootingPose.cmd()
        );

        collectPoseToShootingPose.done().onTrue(
            Commands.parallel(
                prepareShotCommand,
                Commands.waitUntil(prepareShotCommand::isReadyToShoot)
                    .andThen(feed())
            )
        );
        
        return routine;
    }


    private AutoRoutine HubCenterShootRoutine() {
        final AutoRoutine routine = autoFactory.newRoutine("HubCenterShoot");
        final AutoTrajectory startToShootingPose = HubCenterShootTraj.asAutoTraj(routine);
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


    private AutoRoutine HubLeftShootRoutine() {
        final AutoRoutine routine = autoFactory.newRoutine("HubLeftShoot");
        final AutoTrajectory startToShootingPose = HubLeftShootTraj.asAutoTraj(routine);
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


    private AutoRoutine HubRightShootRoutine() {
        final AutoRoutine routine = autoFactory.newRoutine("HubRightShoot");
        final AutoTrajectory startToShootingPose = HubRightShootTraj.asAutoTraj(routine);
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

    
    private AutoRoutine OutpostShootRoutine() {
        final AutoRoutine routine = autoFactory.newRoutine("OutpostShoot");
        final AutoTrajectory startToOutpostPose = OutpostShootTraj$0.asAutoTraj(routine);
        final AutoTrajectory outpostPoseToCollectPose = OutpostShootTraj$1.asAutoTraj(routine);
        final AutoTrajectory collectPoseToShootingPose = OutpostShootTraj$2.asAutoTraj(routine);
        final PrepareShotCommand prepareShotCommand = new PrepareShotCommand(shooter, hood, () -> swerve.getState().Pose);

        routine.active().onTrue(
                Commands.sequence(
                    startToOutpostPose.resetOdometry(),
                    startToOutpostPose.cmd()
                ) 
        );

        startToOutpostPose.active().whileTrue(limelight.idle());
        outpostPoseToCollectPose.active().whileTrue(limelight.idle());
        collectPoseToShootingPose.active().whileTrue(limelight.idle());

        startToOutpostPose.done().onTrue(
            Commands.waitUntil(hanger::isHomed).andThen(
                Commands.sequence(
                    Commands.waitSeconds(0.5),
                    intake.runOnce(() -> intake.set(Intake.Position.INTAKE)),

                    Commands.deadline(
                        outpostPoseToCollectPose.cmd(),
                        Commands.startEnd(
                            () -> intake.set(Intake.Speed.INTAKE), 
                            () -> intake.set(Intake.Speed.STOP),
                            intake
                        )
                    )
                )
            )
        );

        outpostPoseToCollectPose.done().onTrue(
            collectPoseToShootingPose.cmd()
        );

        collectPoseToShootingPose.done().onTrue(
            Commands.parallel(
                prepareShotCommand,
                Commands.waitUntil(prepareShotCommand::isReadyToShoot)
                    .andThen(feed())
            )
        );
        
        return routine;
    }


        private AutoRoutine OutpostShootClimbRoutine() {
        final AutoRoutine routine = autoFactory.newRoutine("OutpostShootClimb");
        final AutoTrajectory startToOutpostPose = OutpostShootTraj$0.asAutoTraj(routine);
        final AutoTrajectory outpostPoseToCollectPose = OutpostShootTraj$1.asAutoTraj(routine);
        final AutoTrajectory collectPoseToShootingPose = OutpostShootTraj$2.asAutoTraj(routine);
        final AutoTrajectory shootingPoseToApproachPose = OutpostShootTraj$3.asAutoTraj(routine);
        final AutoTrajectory approachPoseToClimbPose = OutpostShootTraj$4.asAutoTraj(routine);
        final PrepareShotCommand prepareShotCommand = new PrepareShotCommand(shooter, hood, () -> swerve.getState().Pose);

        routine.active().onTrue(
                Commands.sequence(
                    startToOutpostPose.resetOdometry(),
                    startToOutpostPose.cmd()
                ) 
        );

        startToOutpostPose.active().whileTrue(limelight.idle());
        outpostPoseToCollectPose.active().whileTrue(limelight.idle());
        collectPoseToShootingPose.active().whileTrue(limelight.idle());
        shootingPoseToApproachPose.active().whileTrue(limelight.idle());
        approachPoseToClimbPose.active().whileTrue(limelight.idle());

        startToOutpostPose.done().onTrue(
            Commands.waitUntil(hanger::isHomed).andThen(
                Commands.sequence(
                    Commands.waitSeconds(0.5),
                    intake.runOnce(() -> intake.set(Intake.Position.INTAKE)),

                    Commands.deadline(
                        outpostPoseToCollectPose.cmd(),
                        Commands.startEnd(
                            () -> intake.set(Intake.Speed.INTAKE), 
                            () -> intake.set(Intake.Speed.STOP),
                            intake
                        )
                    )
                )
            )
        );

        outpostPoseToCollectPose.done().onTrue(
            collectPoseToShootingPose.cmd()
        );

        collectPoseToShootingPose.done().onTrue(
            Commands.sequence(
                Commands.parallel(
                    prepareShotCommand,
                    Commands.waitUntil(prepareShotCommand::isReadyToShoot)
                        .andThen(feed())
                ).withTimeout(4.0),
                intake.runOnce(() -> intake.set(Intake.Position.STOWED)),
                shootingPoseToApproachPose.cmd()
            )
        );

        shootingPoseToApproachPose.done().onTrue(
            Commands.sequence(
                hanger.positionCommand(Hanger.Position.HANGING),
                Commands.waitSeconds(0.25),
                approachPoseToClimbPose.cmd()
            )
        );

        approachPoseToClimbPose.done().onTrue(
            hanger.positionCommand(Hanger.Position.HUNG)
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
