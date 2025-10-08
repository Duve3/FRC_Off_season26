// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.commands.IntakeCoral;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DumpRollerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotIntakeSubsystem;

public class RobotContainer {
    public double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.3; // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.5
    ).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController joystick2 = new CommandXboxController(
        1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final ElevatorSubsystem elevator = new ElevatorSubsystem();
    public final PivotIntakeSubsystem pivotSub = new PivotIntakeSubsystem();
    public final DumpRollerSubsystem roller = new DumpRollerSubsystem();

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // Register PathPlanner Named Commands
        // PIVOT INTAKE COMMANDS
        NamedCommands.registerCommand("Deploy Pivot", pivotSub.deployPivot()); // Move pivot to ground intake position
        NamedCommands.registerCommand("Stow Pivot", pivotSub.stowPivot()); // Move pivot to home position
        NamedCommands.registerCommand("Intermediate Pivot", pivotSub.intermediatePivot()); // Move pivot to reef scoring position
        NamedCommands.registerCommand("Collect Coral", pivotSub.collectCoral()); // Deploy, intake until detected, stow
        NamedCommands.registerCommand("Collect and Transfer Coral", pivotSub.collectAndTransferCoral(roller)); // Full auto: collect + transfer to dump
        NamedCommands.registerCommand("Transfer to Dump", pivotSub.transferCoralToDumpRoller(roller)); // Transfer coral to dump roller only
        
        // INTAKE WHEEL COMMANDS
        NamedCommands.registerCommand("Start Intake Wheels", pivotSub.intakeWheels()); // Run intake wheels forward
        NamedCommands.registerCommand("Reverse Intake Wheels", pivotSub.reverseIntakeWheels()); // Run intake wheels backward
        NamedCommands.registerCommand("Stop Intake Wheels", pivotSub.stopWheels()); // Stop intake wheels
        
        // ELEVATOR COMMANDS
        NamedCommands.registerCommand("Raise L0", elevator.setPositionwithThreshold(0)); // Move elevator to level 0
        NamedCommands.registerCommand("Raise L1", elevator.setPositionwithThreshold(1)); // Move elevator to level 1
        NamedCommands.registerCommand("Raise L2", elevator.setPositionwithThreshold(2)); // Move elevator to level 2
        NamedCommands.registerCommand("Raise L3", elevator.setPositionwithThreshold(3)); // Move elevator to level 3
        NamedCommands.registerCommand("Raise L4", elevator.setPositionwithThreshold(4)); // Move elevator to level 4
        
        // DUMP ROLLER COMMANDS
        NamedCommands.registerCommand("Intake Coral", new IntakeCoral(this)); // Run dump roller until current spike detected
        NamedCommands.registerCommand("Drop Coral", roller.dropCoral(0.2).withTimeout(0.5)); // Outtake coral for 0.5s
        NamedCommands.registerCommand("Keep Coral", roller.keepCoral()); // Hold/stop dump roller
        NamedCommands.registerCommand("Prepare Coral Out", roller.PrepareCoral(true)); // Push coral out slightly
        NamedCommands.registerCommand("Prepare Coral In", roller.PrepareCoral(false)); // Pull coral in slightly
        
        // Build auto chooser with PathPlanner
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(joystick.getLeftY(), joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        //elevatorController.a().onTrue(drivetrain.runOnce(() -> eleSubsystem.setPositionwithThreshold(10)));

        //joystick2.b().onTrue(elevator.setPosition(0));
        // Level 1
        //joystick2.rightTrigger().onTrue(new InstantCommand(() -> pivotSub.setPivotSetpoint(-0.4d)));
        //joystick2.leftTrigger().onTrue(new InstantCommand(() -> pivotSub.setPivotSetpoint(0d)));

        joystick2.rightTrigger().onTrue(new InstantCommand(() -> pivotSub.setPivotSetpoint(0.2d)));
        
        // Coral collection and transfer sequence
        // Press POV Up to run the full sequence: deploy -> intake -> stow -> transfer to dump roller
        joystick2.povUp().onTrue(pivotSub.collectAndTransferCoral(roller));
        
        // Mini Test 1: Just collect coral (POV Down)
        joystick2.povDown().onTrue(pivotSub.collectCoral());
        
        // Mini Test 2: Just transfer to dump roller (POV Left) - use when coral already collected
        joystick2.povLeft().onTrue(pivotSub.transferCoralToDumpRoller(roller));
        
        // Dump roller intake
        joystick2.rightBumper().onTrue(new InstantCommand(() -> pivotSub.zeroPositionEncoders()));

        joystick2.leftBumper().onTrue(roller.dropCoral(.5));
        // Level 2
        joystick2.b().onTrue(elevator.setPosition(0));
        //joystick2.a().onTrue(new InstantCommand(pivotSub.setPivotSetpoint(0.05d)));
        // Level 3
        joystick2.a().onTrue(elevator.setPosition(1));
        //joystick2.x().onTrue(new InstantCommand(() -> elevator.setOpenLoop(()->12d)));
        // Level 4
        joystick2.x().onTrue(elevator.setPosition(2));

        joystick2.y().onTrue(elevator.setPosition(3));

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
