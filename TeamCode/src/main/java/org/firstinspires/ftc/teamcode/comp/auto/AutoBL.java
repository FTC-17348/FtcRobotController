package org.firstinspires.ftc.teamcode.comp.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.shared.GlobalConfig;
import org.firstinspires.ftc.teamcode.shared.GlobalConfig.ALLIANCE_POS;
import org.firstinspires.ftc.teamcode.shared.MotionHardware;
import org.firstinspires.ftc.teamcode.shared.MotionHardware.Direction;
import org.firstinspires.ftc.teamcode.shared.VisionHardware;
import org.firstinspires.ftc.teamcode.shared.VisionHardware.PropPosition;

@Config
@Autonomous(name = "Auto - BA Left", group = "Auto")
public class AutoBL extends LinearOpMode {

    public ALLIANCE_POS alliancePos = ALLIANCE_POS.LEFT;

    public GlobalConfig globalConfig = new GlobalConfig(GlobalConfig.AUTONOMOUS_DELIVERY_MODES.DROPPER);
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
            PropPosition propPosition = vision.detectProp();

            switch(propPosition) {
                case MIDDLE:
                    // Dropper Mode
                    robot.moveRobot(.5, -24, 10);
                    robot.dropPixel();

                    robot.moveRobot(.5, 3, 10);
                    robot.turnRobot(Direction.RIGHT, 20, .5, 10);
                    robot.moveRobot(.5, -28, 10);
                    robot.moveArmMotorToPosition(-5000, 10);
                    robot.dropPixelBackBoard();
                    robot.moveRobot(.5, 4, 10);
                    robot.strafeWithTime(.5, 90, 2);
                    robot.moveArmMotorToPosition(0, 1);
                    break;
                case LEFT:
                    //Dropper Mode
                    robot.moveRobot(.5, -16, 10);
                    robot.turnRobot(Direction.LEFT, 10, .8, 10);
                    //robot.moveRobot(.9, -8, 10);
                    robot.dropPixel();

                    robot.moveRobot(.9, 6, 10);
                    robot.turnRobot(Direction.RIGHT, 20, .8, 10);
                    robot.moveRobot(.9, -10, 10);
                    robot.turnRobot(Direction.RIGHT, 11, .8, 10);
                    robot.moveRobot(.9, -22, 10);
                    //iniital position -4520
                    robot.moveArmMotorToPosition(-5000, 10);
                    robot.dropPixelBackBoard();
                    robot.moveRobot(.9, 4, 10);
                    robot.strafeWithTime(.9, 90, 1.5);
                    robot.moveRobot(.9, -2, 10);
                    robot.moveArmMotorToPosition(0, 10);
                    break;
                default:
                    //Dropper Mode
                    robot.moveRobot(.5, -16, 10);
                    robot.turnRobot(Direction.RIGHT, 10, .5, 10);
                    robot.moveRobot(.5, -8, 10);
                    robot.dropPixel();

                    robot.moveRobot(.5, 4, 10);
                    robot.turnRobot(Direction.RIGHT, 17, .5, 10);
                    robot.moveRobot(.5, -15, 10);
                    robot.turnRobot(Direction.LEFT, 4, .5, 10);
                    robot.moveRobot(.5, -11, 10);
                    robot.moveArmMotorToPosition(-5000, 10);
                    robot.dropPixelBackBoard();
                    robot.moveRobot(.9, 4, 10);
                    robot.strafeWithTime(.9, 90, .8);
                    robot.moveRobot(.9, -2, 10);
                    robot.moveArmMotorToPosition(0, 10);
                    break;
            }
            
            sleep(20);
            break;
        }
    }
}
