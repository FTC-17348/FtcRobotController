package org.firstinspires.ftc.teamcode.kuykendall;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Custom TeleOp", group="Linear Opmode")
public class TeleOpMode extends LinearOpMode {

    // Declare OpMode members.
    private SharedCode robot = new SharedCode();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables.
        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            robot.controlLinearExtension(gamepad1);
            robot.drive(gamepad1);
            robot.toggleIntake(gamepad2);
            robot.controlServos(gamepad2);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
