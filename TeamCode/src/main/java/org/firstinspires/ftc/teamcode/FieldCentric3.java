package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//This is the new field centric code from gm0.com
//uses the new IMU module to track rotation/position


@TeleOp
public class FieldCentric3 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");

        boolean endgame = false;

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
 //       frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
 //       backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        Servo rightGrip = hardwareMap.get(Servo.class, "rightGrip");
        Servo leftGrip = hardwareMap.get(Servo.class, "leftGrip");
        Servo wristGrip = hardwareMap.get(Servo.class, "wristGrip");
        Servo launcher = hardwareMap.get(Servo.class, "launcher");



        DcMotor lift = hardwareMap.dcMotor.get("lift");
        DcMotor arm = hardwareMap.dcMotor.get("arm");

        lift.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );
        arm.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );

        double speedMultiplier = .7;

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        //initialize the launch lock
        launcher.setPosition(.2);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
 //here's our loop
            if (gamepad1.y && gamepad1.a) {  //launch drone
                launcher.setPosition(0);
            }

            if (gamepad1.x) {  //declare endgame for arm power boost
                endgame = !endgame;
            }
            telemetry.addData("Endgame?",endgame);
            //test grippers
   //         leftGrip.setPosition(gamepad2.left_trigger);
  //        wristGrip.setPosition(.72);
   //         telemetry.addData("leftposition", gamepad2.left_trigger);
  //          telemetry.addData("wristgrip", gamepad2.right_trigger);


            if (endgame) {
                arm.setPower((gamepad2.left_trigger - gamepad2.right_trigger - .15) * .5);
            } else {
                arm.setPower((gamepad2.left_trigger - gamepad2.right_trigger) * .5);
            }
            lift.setPower((gamepad1.left_trigger-gamepad1.right_trigger)*.90 );


            //commands to operate the grippers
            if (gamepad2.left_bumper) { //operate left gripper
                leftGrip.setPosition(.15);
            } else {
                leftGrip.setPosition(0);
            }

            if (gamepad2.right_bumper) { //operate right gripper
                rightGrip.setPosition(.1);
            } else {
                rightGrip.setPosition(.3);
            }

            if (gamepad2.a) {  //this is the downon the ground position.  bigger is towards the ground
                wristGrip.setPosition(.73);
            }

            if (gamepad2.y) {
                wristGrip.setPosition(.36);
            }

            if (gamepad2.x) {
                wristGrip.setPosition(.64);
            }


            double driveTurn = gamepad1.right_stick_x;
            double gamepadXCoordinate = -gamepad1.left_stick_x; //this simply gives our x value relative to the driver
            double gamepadYCoordinate = -gamepad1.left_stick_y; //this simply gives our y vaue relative to the driver
            double gamepadHypot = Range.clip(Math.hypot(gamepadXCoordinate, gamepadYCoordinate), 0, 1);

            //finds just how much power to give the robot based on how much x and y given by gamepad
            //range.clip helps us keep our power within positive 1
            // also helps set maximum possible value of 1/sqrt(2) for x and y controls if at a 45 degree angle (which yields greatest possible value for y+x)
            double gamepadDegree = Math.atan2(gamepadYCoordinate, gamepadXCoordinate) *57;
            //the inverse tangent of opposite/adjacent gives us our gamepad degree
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            double movementDegree = gamepadDegree + botHeading;

            //adjust the angle we need to move at by finding needed movement degree based on gamepad and robot angles
            double gamepadXControl = Math.cos(Math.toRadians(movementDegree)) * gamepadHypot;
            //by finding the adjacent side, we can get our needed x value to power our motors
            double gamepadYControl = Math.sin(Math.toRadians(movementDegree)) * gamepadHypot;
            //by finding the opposite side, we can get our needed y value to power our motors

            /**
             * again, make sure you've changed the motor names and variables to fit your team
             */

            //by mulitplying the gamepadYControl and gamepadXControl by their respective absolute values, we can guarantee that our motor powers will not exceed 1 without any driveTurn
            //since we've maxed out our hypot at 1, the greatest possible value of x+y is (1/sqrt(2)) + (1/sqrt(2)) = sqrt(2)
            //since (1/sqrt(2))^2 = 1/2 = .5, we know that we will not exceed a power of 1 (with no turn), giving us more precision for our driving
          //  robot.setDrivePower(
                   frontLeftMotor.setPower     ((gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) + driveTurn)*speedMultiplier);
                    frontRightMotor.setPower ((gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) - driveTurn)*speedMultiplier);
                    backLeftMotor.setPower ((gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) + driveTurn)*speedMultiplier);
                    backRightMotor.setPower ((gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) - driveTurn)*speedMultiplier);





            //this is the field centric drive code below
//            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
//            double x = gamepad1.left_stick_x;
//            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.back) {
                imu.resetYaw();
            }

            telemetry.addData("Movement Degree", movementDegree);

            telemetry.addData("BotHeading", "%.3f", botHeading);
            telemetry.addData( "gamepad degree",gamepadDegree);
            telemetry.update();
            // Rotate the movement direction counter to the bot's rotation
 //           double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
 //           double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

 //           rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
   //         double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
   ///         double frontLeftPower = (rotY + rotX + rx) / denominator;
   //         double backLeftPower = (rotY - rotX + rx) / denominator;
    //        double frontRightPower = (rotY - rotX - rx) / denominator;
//            double backRightPower = (rotY + rotX - rx) / denominator;

  //          frontLeftMotor.setPower(frontLeftPower);
    //        backLeftMotor.setPower(backLeftPower);
      //      frontRightMotor.setPower(frontRightPower);
        //    backRightMotor.setPower(backRightPower);
        }
    }
}