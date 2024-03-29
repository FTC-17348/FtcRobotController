package org.firstinspires.ftc.teamcode.kuykendall;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.shared.MotionHardwareRuiming;

@Disabled
@TeleOp(name="BeastOp1", group="Test")
public class BeastOp1 extends LinearOpMode {
    private MotionHardwareRuiming robot = new MotionHardwareRuiming();
    private ElapsedTime runtime = new ElapsedTime();

    private final double pickupPosition = .65;
    private final double dropoffPosition = .2;
    private static final int PICKUP_POSITIONE = 0;
    private static final int DROPOFF_POSITIONE = 350;
    // Modify these constants to represent inches within the 0 to 5 range
    private static final double PICKUP_POSITION_INCHES = 2.0; // Example value
    private static final double DROPOFF_POSITION_INCHES = 4.0; // Example value
    private final double outSlide = .37;
    private final double inSlide = .82;

    private boolean slowmoActive = false;
    private boolean slowmoToggle = false;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Initialization code that was in the init() method
        robot.wristServo.setPosition(0.4);
        robot.rightInt.setPosition(pickupPosition);
        robot.leftInt.setPosition(1.0 - pickupPosition);

        waitForStart();  // Wait for the start button to be pressed

        // Main operation loop
        while (opModeIsActive()) {
            // Code that was in the loop() method
            controlDrive();
            controlLeadScrews();
            controlArm();
            controlServos();
            controlWristAndIntake(); // Control wrist and intake based on the sequence
            controlContinuousServo(); // Control the continuous servo
            updateTelemetry();
        }
    }

    private void controlDrive() {
        double strafe = -gamepad2.left_stick_x;
        double rotate = -gamepad2.right_stick_x;
        double drive = gamepad2.left_stick_y;

        if (gamepad2.left_stick_button && !slowmoToggle) {
            slowmoActive = !slowmoActive;
            slowmoToggle = true;
        } else if (!gamepad2.left_stick_button) {
            slowmoToggle = false;
        }

        double speedModifier = slowmoActive ? 0.25 : 1.0;
        drive *= speedModifier;
        strafe *= speedModifier;
        rotate *= speedModifier;

        double frontLeftPower = drive + strafe + rotate;
        double frontRightPower = drive - strafe - rotate;
        double backLeftPower = drive - strafe + rotate;
        double backRightPower = drive + strafe - rotate;

        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        robot.frontLeftMotor.setPower(frontLeftPower);
        robot.frontRightMotor.setPower(frontRightPower);
        robot.backLeftMotor.setPower(backLeftPower);
        robot.backRightMotor.setPower(backRightPower);
    }
    private void controlLeadScrews() {
        if (gamepad1.dpad_up) {
            robot.leftLeadScrew.setPower(0.6);  // Set power to raise the robot
            robot.rightLeadScrew.setPower(0.6);

        } else {
            robot.leftLeadScrew.setPower(0);  // Set power to zero when not pressed
            robot.rightLeadScrew.setPower(0);
        }
        if (gamepad1.dpad_down) {
            robot.leftLeadScrew.setPower(-0.6);  // Set power to raise the robot
            robot.rightLeadScrew.setPower(-0.6);
        } else {
            robot.leftLeadScrew.setPower(0);  // Set power to zero when not pressed
            robot.rightLeadScrew.setPower(0);
        }
    }

    private void controlArm() {
        robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // When Y button is pressed, move the arm up
        if (gamepad1.y) {
            robot.armMotor.setPower(1.0);
        }
        // When A button is pressed, move the arm down
        else if (gamepad1.a) {
            robot.armMotor.setPower(-1.0);
        }
        // When neither Y nor A is pressed, stop the arm
        else {
            robot.armMotor.setPower(0);
        }

        robot.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /*private void controlArm() {
        robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (gamepad1.a) {
            robot.armMotor.setTargetPosition(PICKUP_POSITIONE);
            robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armMotor.setPower(-1.0);
        } else if (gamepad1.y) {
            robot.armMotor.setTargetPosition(DROPOFF_POSITIONE);
            robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armMotor.setPower(1.0);
        }

        if (gamepad1.dpad_left) {
            robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        robot.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (!robot.armMotor.isBusy()) {
            robot.armMotor.setPower(0);
        }
    } */

        /*if (gamepad1.a) {
            robot.setArmPosition(1.0, PICKUP_POSITION_INCHES, 5.0, runtime, telemetry, this::opModeIsActive);
        } else if (gamepad1.y) {
            robot.setArmPosition(1.0, DROPOFF_POSITION_INCHES, 5.0, runtime, telemetry, this::opModeIsActive);
        }

        if (gamepad1.dpad_left) {
            robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        robot.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }*/

    private void controlServos() {
        if (gamepad1.right_bumper) {
            robot.bucketServo.setPosition(inSlide);
        } else if (gamepad1.left_bumper) {
            robot.bucketServo.setPosition(outSlide);
        }
    }
    private void controlWristAndIntake() {
        // Check the current step in the sequence
        switch(robot.sequenceStep) {
            case 0:
                if (gamepad2.y) {
                    robot.wristServo.setPosition(.8); // Adjust to your wrist up position
                    robot.sequenceStep = 1;
                    robot.stepStartTime = System.currentTimeMillis();
                }
                break;

            case 1:
                if (System.currentTimeMillis() - robot.stepStartTime >= robot.stepDuration) {
                    robot.rightInt.setPosition(pickupPosition);
                    robot.leftInt.setPosition(1.0 - pickupPosition);
                    robot.sequenceStep = 0;
                }
                break;

            // ... [Implement other steps as needed]

            default:
                break;
        }

        if (gamepad2.a) {
            robot.rightInt.setPosition(dropoffPosition);
            robot.leftInt.setPosition(1.0 - dropoffPosition);
            robot.wristServo.setPosition(0.4); // Wrist down position
        }
    }

    private void controlContinuousServo() {
        // Updated logic for continuous servo control
        if (gamepad1.x) {
            robot.intServo.setPower(2.0); // Spin forward
        } else if (gamepad1.b) {
            robot.intServo.setPower(-2.0); // Spin backward
        } else {
            robot.intServo.setPower(0); // Stop spinning
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Wrist Servo Position", robot.wristServo.getPosition());
        telemetry.addData("Right Intake Servo Position", robot.rightInt.getPosition());
        telemetry.addData("Left Intake Servo Position", robot.leftInt.getPosition());
        telemetry.addData("Arm Position", robot.armMotor.getCurrentPosition());
        telemetry.update();
    }
}