package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Trajectory Test")
public class TrajectoryTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize TankDrive with starting pose
        TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0.0, -60.0, Math.toRadians(90.0)));
        TelemetryPacket telemetryPacket = new TelemetryPacket();
        int upPosition = 4700;
        int downPosition = 500;
        // Wait for start
        waitForStart();

        // Build trajectory using your TankDrive's action builder
        Pose2d startPose = drive.localizer.getPose();
        TrajectoryActionBuilder builder = drive.actionBuilder(startPose);

        Action followTrajectory = builder
                .splineTo(new Vector2d(0.0, 0.0), Math.toRadians(90))
                .splineTo(new Vector2d(-16.0, 0.0), Math.toRadians(180))
                .splineTo(new Vector2d(-16.0, -60.0), Math.toRadians(270))
                .splineTo(new Vector2d(0.0, -60.0), Math.toRadians(360))
                .build();

        drive.setPoseEstimate(drive.localizer.getPose());

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive() && followTrajectory.run(telemetryPacket)) {
            double elapsed = timer.seconds();

            // Example: open arm servo after 1 second
            if (elapsed > 1) {
                drive.armServo.setPower(1);  // rotate forward slowly
                drive.SpinServo.setPosition(230.0 / 300.0);
            }

            if (elapsed < 1) {
                // Go up
                drive.armMotor.setTargetPosition(upPosition);
                drive.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.armMotor.setPower(1);
            } else if (elapsed >= 1 && elapsed < 5) {
                // Hold position (motor stays at target)
                drive.armMotor.setPower(0.3); // low holding power
            } else if (elapsed >= 5 && elapsed < 8) {
                // Come back down
                drive.armMotor.setTargetPosition(downPosition);
                drive.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.armMotor.setPower(0.5);
            } else {
                // Stop motor
                drive.armMotor.setPower(0);
                drive.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // optional reset
            }

            telemetry.addData("Status", "Following trajectory...");
            telemetry.addData("Elapsed Time", elapsed);
            telemetry.update();

            sleep(10);
        }

        telemetry.addData("Status", "Trajectory complete");
        telemetry.update();
    }
}