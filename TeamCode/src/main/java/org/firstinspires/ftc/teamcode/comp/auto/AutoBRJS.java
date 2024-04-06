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
@Autonomous(name = "Auto - BA Right No Back", group = "Auto")
public class AutoBRJS extends LinearOpMode {

    public ALLIANCE_POS alliancePos = ALLIANCE_POS.RIGHT;

    public GlobalConfig globalConfig = new GlobalConfig(GlobalConfig.AUTONOMOUS_DELIVERY_MODES.DROPPER);
    MotionHardware robot = new MotionHardware(this, globalConfig);
    VisionHardware vision = new VisionHardware(this, alliancePos);
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();
        vision.init();

        robot.moveArm(.5, 5, 5);

        waitForStart();

        //TODO Add Park from BR

        // Center Strike Line
        // - Forward: 31.75
        // - Reverse: -44.75
        while(opModeIsActive()) {
            PropPosition propPosition = vision.detectProp();

            switch(propPosition) {
                case MIDDLE:
                    // Dropper Mode
                    robot.dropperDown();
                    robot.moveRobot(.75, -24, 10);
                    robot.dropperUp();
                    break;
                case RIGHT:
                    //Dropper Mode
                    robot.dropperDown();
                    robot.moveRobot(.75, -24, 2);
                    robot.turnRobot(Direction.RIGHT, 18.5, .65, 2);
                    robot.moveRobot(.5, 2, 2);
                    robot.dropperUp();
                    break;
                default:
                    robot.dropperDown();
                    robot.moveRobot(.75, -24, 2);
                    robot.turnRobot(Direction.LEFT, 18.5, .5, 2);
                    robot.moveRobot(.75,-6,10);
                    robot.dropperUp();
            }

            sleep(20);
            break;
        }
    }
}