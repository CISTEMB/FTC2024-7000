package org.firstinspires.ftc.teamcode;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.commands.ElevatorV2ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.WormSetPowerCommand;
import org.firstinspires.ftc.teamcode.commands.WristSetAngleCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorV2;
import org.firstinspires.ftc.teamcode.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.subsystems.Worm;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="Autonomous: TestSpecimen", group="000Real")
public class Autonomous_TestSpecimen extends CommandOpMode {

    private ElevatorV2 elevatorV2;
    private Worm worm;
    private Grabber grabber;
    private Wrist wrist;
    private SampleMecanumDrive drive;
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

        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(26.16, -62.72, Math.toRadians(90.00)))
                .splineTo(new Vector2d(4.64, -36.42), Math.toRadians(90.00))
                .build();
        drive.setPoseEstimate(trajectory0.start());
        drive.followTrajectorySequence(trajectory0);

        SequentialCommandGroup scoreSpecimenGroup = new SequentialCommandGroup(
                new WormSetPowerCommand(worm, 1).interruptOn(() -> worm.getAngle() > 23.00),
                new WormSetPowerCommand(worm, 0.2).interruptOn(() -> worm.getAngle() > 27.5), //final target: 28.5
                new ElevatorV2ExtendCommand(elevatorV2, 1).interruptOn(() -> elevatorV2.getDistanceInInches() >= 11.0),
                new ElevatorV2ExtendCommand(elevatorV2, 0.5).interruptOn(() -> elevatorV2.getDistanceInInches() >= 12.8),
                new WristSetAngleCommand(wrist, 132.0) //132.0
        );

        schedule(scoreSpecimenGroup);

        TrajectorySequence pushForward = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(5)
                .build();
        drive.setPoseEstimate(pushForward.start());
        drive.followTrajectorySequence(pushForward);
    }

    @Override
    public void run() {
        super.run();
        elevatorV2.SetWormAngle(worm.getAngle()); //set this continually so elevator can know how far it can go
        worm.SetElevatorDistanceInInches(elevatorV2.getHorizontalExtension());
        telemetry.update();
    }
}
