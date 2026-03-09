// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Collections;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AllignToPoseCommand;
import frc.robot.commands.ShooterTuning;
import frc.robot.commands.TurretAllignCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

// I changed the inversion of the encoder and steer motors for the swerve and we fliped the pigeon orientation and changed 

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity
    private SlewRateLimiter rateLimitX = new SlewRateLimiter(12);
    private SlewRateLimiter rateLimitY = new SlewRateLimiter(12);
    private SlewRateLimiter rateLimitRot = new SlewRateLimiter(36);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    private final CommandXboxController joystickOtherForIntake = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final TurretSubsystem turret = new TurretSubsystem(joystick);

    public final IntakeSubsystem intake = new IntakeSubsystem(joystick);

    public final IndexerSubsystem indexer = new IndexerSubsystem();

    public final ShooterSubsystem shooter = new ShooterSubsystem(joystick);

     

    // private final TurretSubsystem m_turret = new TurretSubsystem(joystick);

    public RobotContainer() {
        configureBindings();

        SmartDashboard.putNumber("turret positon", 0);
        SmartDashboard.putNumber("Hood Angle", 0);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(rateLimitX.calculate(-joystick.getLeftY() * MaxSpeed)) // Drive
                                                                                                                         // forward
                                                                                                                         // with
                                                                                                                         // negative
                                                                                                                         // Y
                                                                                                                         // (forward)
                        .withVelocityY(rateLimitY.calculate(-joystick.getLeftX() * MaxSpeed)) // Drive left with
                                                                                              // negative X (left)
                        .withRotationalRate(rateLimitRot.calculate(-joystick.getRightX() * MaxAngularRate)) // Drive
                                                                                                            // counterclockwise
                                                                                                            // with
                                                                                                            // negative
                                                                                                            // X (left)
                ));

        joystick.a().onTrue(intake.intakeCommand(Rotations.of(1.25), 0.0));
        joystick.a().whileFalse(intake.intakeCommand(Rotations.of(0.0), 0.0));

        indexer.setDefaultCommand(indexer.Stop());
        shooter.setDefaultCommand(shooter.shootCommand(RotationsPerSecond.of(0)));
       // joystick.b().whileTrue(indexer.kickCommand(RotationsPerSecond.of(-5)));
        joystick.x().whileTrue(shooter.shootCommand(RotationsPerSecond.of(30)));
        joystick.y().whileTrue(new TurretAllignCommand(turret, drivetrain,()-> drivetrain.getState().Speeds));
        //[joystick.b().whileTrue(new AllignToPoseCommand(drivetrain, joystick));
        //joystick.y().whileTrue(turret.setTurretCommand(Rotations.of(0.4)));
        //turret.setDefaultCommand(new TurretAllignCommand(turret, drivetrain, ()-> drivetrain.getState().Speeds));
        // joystickOtherForIntake.b().whileFalse(indexer.kickCommand(RotationsPerSecond.of(0)));
        // joystickOtherForIntake.x().whileFalse(
        // shooter.shootCommand(RotationsPerSecond.of(0)));
        
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        // point.withModuleDirection(new Rotation2d(-joystick.getLeftY(),
        // -joystick.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(new InstantCommand(() -> drivetrain.resetPose(new Pose2d())));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // joystick.a().whileTrue(new
        // RunCommand(()->m_turret.setTurretPosition(Angle.ofBaseUnits(SmartDashboard.getNumber("turret
        // pose", 0),Rotation),0), m_turret));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
                // Reset our field centric heading to match the robot
                // facing away from our alliance station wall (0 deg).
                drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                // Then slowly drive forward (away from us) for 5 seconds.
                drivetrain.applyRequest(() -> drive.withVelocityX(0.5)
                        .withVelocityY(0)
                        .withRotationalRate(0))
                        .withTimeout(5.0),
                // Finally idle for the rest of auton
                drivetrain.applyRequest(() -> idle));

    }

    public void smallestAngleToTurrent(double setPoint) {

        ArrayList<Double> angles = new ArrayList<Double>();
        ArrayList<Double> diffs = new ArrayList<Double>();
        for (int i = -2; i < 3; i++) {
            double testAngle = setPoint + i;
            if ((-1.2 <= testAngle) && (testAngle <= 1.2)) {
                angles.add(testAngle);
                double currentTargetDiff = Math.abs(turret.getTurret().getPosition().in(Rotations) - testAngle);
                diffs.add(currentTargetDiff);
            }
        }
        int indexOfMin = diffs.indexOf(Collections.min(diffs));

        turret.getTurret().setPosition(Rotations.of(angles.get(indexOfMin)));
    }
}
