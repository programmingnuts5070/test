package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;


public class TeleOpDriveTrain extends LinearOpMode {

    DcMotor linearSlideMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        linearSlideMotor = hardwareMap.get(DcMotor.class, "linearSlideMotor");
        linearSlideMotor.setDirection(DcMotor.Direction.FORWARD);
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();






        // Run this loop until the OpMode is stopped
        while (opModeIsActive()) {
            // Set the drive powers based on gamepad inputs
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y, // Forward/backward movement
                            -gamepad1.left_stick_x  // Strafing left/right
                    ),
                    -gamepad1.right_stick_x       // Rotation
            ));
            drive.updatePoseEstimate();

            if(gamepad1.right_trigger > 0) {
                linearSlideMotor.setPower(gamepad1.right_trigger);
            }
            else if (gamepad1.left_trigger > 0) {
                linearSlideMotor.setPower(-gamepad1.left_trigger);
            }
            else {
                linearSlideMotor.setPower(0);
            }

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", drive.pose.heading);
            telemetry.update();
        }



    }
}
