package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class Limelight extends LinearOpMode {

    private DcMotorEx leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor;
    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors
        leftFrontMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBackMotor = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBackMotor = hardwareMap.get(DcMotorEx.class, "rightBack");

        // Configure motor directions & modes
        leftFrontMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double xOffset = result.getTx();  // Left/Right offset
                double yOffset = result.getTy();  // Forward/Backward offset
                telemetry.addData("xOffset", xOffset);
                telemetry.addData("yOffset", yOffset);
                telemetry.update();
                // Define stopping condition (stop before getting too close)
                if (Math.abs(yOffset) < -6) { // Stop if we are within 5 degrees of target
                    stopMotors();
                    limelight.stop(); // Turn off camera to stop tracking
                    break; // Exit loop
                }

                // Adjust movement toward target
                double turnPower = -0.003 * (xOffset-0); // Turning power
                double forwardPower = 0.03 * (yOffset-0); // Forward speed

                drive(forwardPower, turnPower);
            } else {
                stopMotors(); // Stop if no target detected
            }

//            telemetry.addData("xOffset", result.getTx());
//            telemetry.addData("yOffset", result.getTy());
//            telemetry.update();
        }
    }

    private void drive(double forward, double turn) {
        leftFrontMotor.setPower(forward + turn);
        rightFrontMotor.setPower(forward - turn);
        leftBackMotor.setPower(forward + turn);
        rightBackMotor.setPower(forward - turn);
    }

    private void stopMotors() {
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }
}
