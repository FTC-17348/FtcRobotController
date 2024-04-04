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
                    robot.moveRobot(.7, -24, 10);
                    robot.dropperUp();


                    robot.moveRobot(.75, 5, 10);
                    robot.turnRobot(Direction.RIGHT, 18.5, .5, 10);
                    robot.moveRobot(.75, 31, 10);
                    robot.moveArmMotorToPosition(-900, 10);
                    robot.dropPixelBackBoard();
                    sleep(500);
                    robot.moveArmMotorToPosition(-1200,10);
                    robot.moveRobot(.75, -6, 10);
                    robot.strafeWithTime(.5,270,1.5);
                    robot.turnRobot(Direction.LEFT, 37, .5, 10);
                    robot.moveRobot(.5,-10,10);
                    robot.wristDown();
                    robot.moveArmMotorToPosition(0, 1);
                    break;
                case LEFT:
                    robot.moveRobot(.75, -24, 2);
                    robot.turnRobot(Direction.LEFT, 18.5, .5, 2);
                    robot.moveRobot(.5, 2, 2);
                    robot.dropperUp();


                    robot.moveArmMotorToPosition(-400, 10);
                    robot.moveRobot(.75, -20, 2);
                    robot.turnRobot(Direction.RIGHT,37,.65,2);
                    robot.moveRobot(.6,8,2);
                    robot.strafeWithTime(.6,90,.7);
                    robot.moveArmMotorToPosition(-1000, 10);
                    robot.moveRobot(.6,3.5,2);
                    robot.dropPixelBackBoard();
                    sleep(500);
                    robot.moveArmMotorToPosition(-1200, 10);
                    robot.moveRobot(.5,-4,2);
                    robot.strafeWithTime(.5,90,1.4);
                    robot.turnRobot(Direction.LEFT, 37, .65, 2);
                    break;
                default:
                    //Dropper Mode
                    robot.moveRobot(.6, -24, 2);
                    robot.turnRobot(Direction.RIGHT, 18.5, .5, 2);
                    robot.moveRobot(.75,-6,2);
                    robot.dropperUp();


                    robot.moveRobot(.75, 37, 2);
                    robot.strafeWithTime(.5,270,.75);
                    robot.moveArmMotorToPosition(-800, 10);
                    robot.dropPixelBackBoard();
                    sleep(500);
                    robot.moveArmMotorToPosition(-1200,10);
                    robot.moveRobot(.75, -4, 10);
                    robot.strafeWithTime(.5, 270, 1.4);
                    robot.turnRobot(Direction.LEFT,37,.5,10);
                    robot.moveRobot(.75,-8,10);
                    break;
            }
            
            sleep(20);
            break;
        }
    }
}
