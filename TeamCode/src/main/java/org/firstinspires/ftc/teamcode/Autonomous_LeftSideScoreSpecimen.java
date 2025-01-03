package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.ElevatorV2ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorV2RetractCommand;
import org.firstinspires.ftc.teamcode.commands.GrabberDropCommand;
import org.firstinspires.ftc.teamcode.commands.GrabberPickupCommand;
import org.firstinspires.ftc.teamcode.commands.WormSetPowerCommand;
import org.firstinspires.ftc.teamcode.commands.WristSetAngleCommand;
import org.firstinspires.ftc.teamcode.commands.roadrunner.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorV2;
import org.firstinspires.ftc.teamcode.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Worm;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

@Autonomous(name="Autonomous: LeftSideScoreSpecimen", group="000Real")
public class Autonomous_LeftSideScoreSpecimen extends CommandOpMode {

    private ElevatorV2 elevatorV2;
    private Worm worm;
    private Grabber grabber;
    private Wrist wrist;
    private SampleMecanumDrive drive;
    private MecanumDriveSubsystem mecanumDriveSubsystem;
    private Climber climber;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("intialized", "true");
        elevatorV2 = new ElevatorV2(hardwareMap, telemetry);
        worm = new Worm(hardwareMap, telemetry);
        grabber = new Grabber(hardwareMap, telemetry);
        wrist = new Wrist(hardwareMap, telemetry, true);
        wrist.SetSpeed(0.5);
        climber = new Climber(hardwareMap, telemetry);
        climber.Goto(0);

        drive = new SampleMecanumDrive(hardwareMap);
        mecanumDriveSubsystem = new MecanumDriveSubsystem(drive, false);

        //place the robot just to the left of the right tape parking area
        Trajectory leftTapeToScoringPosition = mecanumDriveSubsystem.trajectoryBuilder(new Pose2d(-38.76, -63.58, Math.toRadians(90.00)))
                .lineToConstantHeading(new Vector2d(-11.64, -61.58))
                .build();

        drive.setPoseEstimate(leftTapeToScoringPosition.start());

        SequentialCommandGroup scoreSpecimenGroup = new SequentialCommandGroup(
                new WormSetPowerCommand(worm, 1).interruptOn(() -> worm.getAngle() > 23.00),
                new WormSetPowerCommand(worm, 0.2).interruptOn(() -> worm.getAngle() > 27.0), //final target: 28.5
                new ParallelCommandGroup(
                    new ElevatorV2ExtendCommand(elevatorV2, 1).interruptOn(() -> elevatorV2.getDistanceInInches() >= 11.0),
                    new WristSetAngleCommand(wrist, 132.0) //132.0
                ),
                new ElevatorV2ExtendCommand(elevatorV2, 0.5).interruptOn(() -> elevatorV2.getDistanceInInches() >= 12.8)
        );

        SequentialCommandGroup resetElevatorGroup = new SequentialCommandGroup(
                new ElevatorV2RetractCommand(elevatorV2, 1).interruptOn(() -> elevatorV2.getDistanceInInches() <= 2),
                new ElevatorV2RetractCommand(elevatorV2, 0.2).interruptOn(() -> elevatorV2.getDistanceInInches() <= 2),
                new WormSetPowerCommand(worm, -1).interruptOn(() -> worm.getAngle() <= -5)
        );

        Trajectory pushForward = mecanumDriveSubsystem.trajectoryBuilder(leftTapeToScoringPosition.end())
                .lineToConstantHeading(new Vector2d(-12.64, -37.69), new AngularVelocityConstraint(6), new ProfileAccelerationConstraint(6))
                .build();

        Trajectory backup = mecanumDriveSubsystem.trajectoryBuilder(pushForward.end())
                .lineToConstantHeading(new Vector2d(-12.64, -47.69))
                .build();

        Trajectory driveToFirstSample = mecanumDriveSubsystem.trajectoryBuilder(backup.end())
                .lineToConstantHeading(new Vector2d(-52.87, -47.69))
                .build();

        Trajectory pickupToFirstSample = mecanumDriveSubsystem.trajectoryBuilder(driveToFirstSample.end())
                .lineToConstantHeading(new Vector2d(-52.87, -35.00), new AngularVelocityConstraint(6), new ProfileAccelerationConstraint(6))
                .build();

        schedule(
            new SequentialCommandGroup(
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, leftTapeToScoringPosition),
                scoreSpecimenGroup,
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, pushForward),
                new ParallelDeadlineGroup(
                    new TrajectoryFollowerCommand(mecanumDriveSubsystem, backup),
                    new GrabberPickupCommand(grabber)
                ),
                resetElevatorGroup,
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, driveToFirstSample),
                new ParallelCommandGroup(
                    new WristSetAngleCommand(wrist, 150),
                    new ElevatorV2ExtendCommand(elevatorV2, 0.2).interruptOn(() -> elevatorV2.getDistanceInInches() >= 4)
                ),
                new ParallelDeadlineGroup(
                    new TrajectoryFollowerCommand(mecanumDriveSubsystem, pickupToFirstSample),
                    new GrabberDropCommand(grabber)
                )
            )
        );
    }

    @Override
    public void run() {
        super.run();
        elevatorV2.SetWormAngle(worm.getAngle()); //set this continually so elevator can know how far it can go
        worm.SetElevatorDistanceInInches(elevatorV2.getHorizontalExtension());
        telemetry.update();
    }
}
