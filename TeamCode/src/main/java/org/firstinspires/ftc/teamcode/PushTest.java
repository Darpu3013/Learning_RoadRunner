package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Push Test Loop")
public class PushTest extends LinearOpMode {
    Pose2d startPose = new Pose2d(0, 0, 0);
    TankDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new TankDrive(hardwareMap, startPose);

        waitForStart();
        if (isStopRequested()) return;

        // Initial pose estimate
        drive.setPoseEstimate(startPose);

        while (opModeIsActive()) {
            // Move forward
            Action forward = drive.actionBuilder(drive.localizer.getPose())
                    .lineToX(30)
                    .build();
            Actions.runBlocking(forward);

            // Move back to start
            Action backward = drive.actionBuilder(drive.localizer.getPose())
                    .lineToX(0)
                    .build();
            Actions.runBlocking(backward);
        }
    }
}
