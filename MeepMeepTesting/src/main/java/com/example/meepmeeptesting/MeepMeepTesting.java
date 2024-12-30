package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.DriveTrainType;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
//import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.SequenceSegment;
//import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment;
//import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TurnSegment;
//import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.WaitSegment;


public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

//        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(26.16, -62.72, Math.toRadians(90.00)))
//                .splineTo(new Vector2d(4.64, -36.42), Math.toRadians(90.00))
//                .build();


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(26.16, -62.72, Math.toRadians(90.00)))
                        .splineTo(new Vector2d(4.64, -36.42), Math.toRadians(90.00))
                        .build());

        myBot.setDriveTrainType(DriveTrainType.MECANUM);

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}