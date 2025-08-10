package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Marker Test")
public class MarkerTest extends LinearOpMode {

    private TankDrive drive;
    private final int upPos = 4500;
    private final int downPos = 700;
    public Action pickup() {
        return (packet) -> {
            drive.armMotor.setTargetPosition(upPos);
            drive.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.armMotor.setPower(1);
            drive.SpinServo.setPosition(230.0 / 360.0);
            drive.armServo.setPower(1);
            return true;
        };
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize TankDrive with starting pose
        TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0.0, -50.0, Math.toRadians(90.0)));
        TelemetryPacket telemetryPacket = new TelemetryPacket();

        // Wait for start
        waitForStart();

        // Build trajectory using your TankDrive's action builder
        Pose2d startPose = drive.localizer.getPose();
        TrajectoryActionBuilder builder = drive.actionBuilder(startPose);

        Action followTrajectory = builder
                .lineToY(0.0)
                .stopAndAdd(new SequentialAction(pickup()))
                .splineTo(new Vector2d(-8, 3), Math.toRadians(161.05))
                .splineTo(new Vector2d(-15, 2.7), Math.toRadians(210.70))
                .splineTo(new Vector2d(-20, -30), Math.toRadians(250.56))
                .build();




/*
                .splineTo(new Vector2d(0.0, 0.0), Math.toRadians(90)) // approx 72 inches
                .splineTo(new Vector2d(-16.0, 0.0), Math.toRadians(180))
                .afterDisp(10, () -> {
                    drive.armServo.setPower(1);
                    drive.SpinServo.setPosition(230.0 / 360.0);
                    drive.armMotor.setPower(1);
                })
                .splineTo(new Vector2d(-16.0, -60.0), Math.toRadians(270))
                .splineTo(new Vector2d(0.0, -60.0), Math.toRadians(360))
                build();
        /*
        .afterDisp(0, () -> {
                    drive.armServo.setPower(1);
                    drive.SpinServo.setPosition(230.0 / 360.0);
                    drive.armMotor.setTargetPosition(upPos);
                    drive.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.armMotor.setPower(1);
                })
                 .afterDisp(0, () -> {
                    drive.armMotor.setTargetPosition(downPos);
                    drive.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.armMotor.setPower(1);
                })
 */


        drive.setPoseEstimate(drive.localizer.getPose());


        while (opModeIsActive() && followTrajectory.run(telemetryPacket)) {
            Pose2d pose = drive.localizer.getPose();
            telemetry.addData("X", pose.position.x);
            telemetry.addData("Y", pose.position.y);
            telemetry.update();
            idle();
        }


    }

    }
