package org.firstinspires.ftc.teamcode.comp.auto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.shared.GlobalConfig;
import org.firstinspires.ftc.teamcode.shared.GlobalConfig.ALLIANCE_POS;
import org.firstinspires.ftc.teamcode.shared.GlobalConfig.ALLIANCE_COL;
import org.firstinspires.ftc.teamcode.shared.MotionHardware;
import org.firstinspires.ftc.teamcode.shared.MotionHardware.Direction;
import org.firstinspires.ftc.teamcode.shared.VisionHardware;
import org.firstinspires.ftc.teamcode.shared.VisionHardware.PropPosition;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
@Autonomous(name = "Auto - BA Right", group = "Auto")
public class AutoBR extends LinearOpMode {

    public ALLIANCE_POS alliancePos = ALLIANCE_POS.RIGHT;
    public ALLIANCE_COL allianceCol = ALLIANCE_COL.RED;
    public GlobalConfig globalConfig = new GlobalConfig(GlobalConfig.AUTONOMOUS_DELIVERY_MODES.DROPPER);

    private static int DESIRED_TAG_ID = 0;

    private static int DESIRED_DISTANCE = 6;

    private AprilTagDetection desiredTag = null;
    MotionHardware robot = new MotionHardware(this, globalConfig);
    VisionHardware vision = new VisionHardware(this, alliancePos);
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();
        vision.init();

        robot.moveArm(.5, 5, 5);

        waitForStart();

        //TODO In parking code, drop arms to TeleOp pickup position

        // Center Strike Line
        // - Forward: 31.75
        // - Reverse: -44.75
        while(opModeIsActive()) {
            vision.resetExposure();
            PropPosition propPosition = vision.detectProp();
            // Left = 4, Middle = 5, Right = 6

            switch(propPosition) {
                case MIDDLE:
                    robot.dropperDown();
                    robot.moveRobot(.7, -24, 10);
                    robot.dropperUp();
                    robot.moveRobot(.7, 22, 10);
                    robot.turnRobot(Direction.RIGHT, 18.5, .5, 10);
                    robot.moveRobot(.7, 70, 10);
                    robot.strafeWithTime(.5,270,1.92);
                    robot.moveRobot(.6, 9, 2);
                    robot.moveArmMotorToPosition(-1100,2);
                    robot.dropPixelBackBoard();
                    sleep(400);
                    robot.moveArmMotorToPosition(-1400,2);
                    robot.moveRobot(.5,-4,2);
                    robot.strafeWithTime(.5,270,1.45);
                    robot.turnRobot(Direction.LEFT,37,.5,2);
                    robot.moveRobot(.5,-5,2);
                    break;
                case LEFT:
                    robot.dropperDown();
                    robot.moveRobot(.75, -24, 2);
                    robot.turnRobot(Direction.LEFT, 18.5, .5, 2);
                    robot.moveRobot(.5, 2, 2);
                    robot.dropperUp();
                    robot.strafeWithTime(.5,90,1.7);
                    robot.moveRobot(.75,70,2);
                    robot.strafeWithTime(.5,270,2.25);
                    robot.moveRobot(.6, 12, 2);
                    robot.moveArmMotorToPosition(-1100,2);
                    robot.dropPixelBackBoard();
                    sleep(400);
                    robot.moveArmMotorToPosition(-1400,2);
                    robot.moveRobot(.5,-4,2);
                    robot.strafeWithTime(.5,270,1.35);
                    robot.turnRobot(Direction.LEFT,37,.5,2);
                    robot.moveRobot(.5,-5,2);
                    break;
                default:
                    //Dropper Mode
                    robot.dropperDown();
                    robot.moveRobot(.6, -24, 2);
                    robot.turnRobot(Direction.RIGHT, 18.5, .5, 2);
                    robot.moveRobot(.75,-6,2);
                    robot.dropperUp();
                    robot.moveRobot(.75,4,2);
                    robot.strafeWithTime(.5,270,1.8);
                    robot.moveRobot(.75,-65,2);
                    robot.strafeWithTime(.5,90,1.375);
                    robot.turnRobot(Direction.LEFT,37,.5,2);
                    robot.moveRobot(.5,13,2);
                    robot.moveArmMotorToPosition(-1000,2);
                    robot.dropPixelBackBoard();
                    sleep(400);
                    robot.moveArmMotorToPosition(-1400,2);
                    robot.moveRobot(.5,-4,2);
                    robot.strafeWithTime(.5,90,1.55);
                    robot.turnRobot(Direction.LEFT,37,.5,2);
                    robot.moveRobot(.5,-5,2);
                    break;
            }

            sleep(20);
            break;
        }
    }
}
