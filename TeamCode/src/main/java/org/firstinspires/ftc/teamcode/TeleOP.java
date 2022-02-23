package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class TeleOP extends OpMode {
    private DcMotorEx leftFront, leftBack, rightFront, rightBack, intake, outtake, delivery1, delivery2;
    private Servo duckLeft,duckRight, pully;
    private boolean direction, togglePrecision, reverse;
    private double factor;
    private BNO055IMU imu;
    @Override

    //FL goes to port 0, BL goes to port 1, BR goes to port 2, FR goes to port 3

    public void init() {
        //Maps all the variables to its respective hardware
        leftFront = (DcMotorEx) hardwareMap.dcMotor.get("FL");
        leftBack = (DcMotorEx) hardwareMap.dcMotor.get("BL");
        rightFront = (DcMotorEx) hardwareMap.dcMotor.get("FR");
        rightBack = (DcMotorEx) hardwareMap.dcMotor.get("BR");
        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        delivery1 = (DcMotorEx) hardwareMap.dcMotor.get("delivery1");
        delivery2 = (DcMotorEx) hardwareMap.dcMotor.get("delivery2");
        outtake = (DcMotorEx) hardwareMap.dcMotor.get("outtake");
        duckLeft = (Servo) hardwareMap.get("duckLeft");
        duckRight = (Servo) hardwareMap.get("duckRight");
        pully = (Servo) hardwareMap.get("pully");
        //Initialize all the hardware to use Encoders
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        delivery1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        delivery2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Initialize the motors to begin stationary
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        delivery1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        delivery2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Left Motors are in reverse and Right Motors are forward so the robot can move forward
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        delivery1.setDirection(DcMotorSimple.Direction.FORWARD);
        delivery2.setDirection(DcMotorSimple.Direction.FORWARD);
        outtake.setDirection(DcMotorSimple.Direction.FORWARD);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        togglePrecision = false;
        reverse = false;
    }

    @Override
    public void loop() {
        if(gamepad1.start)
            togglePrecision = !togglePrecision;
        if(gamepad1.back)
            reverse = !reverse;
        if(gamepad1.b) {
            intake.setPower(-1);
            delivery1.setPower(1);
            delivery2.setPower(-1);
        }
        else if(gamepad1.a) {
            intake.setPower(1);
            delivery1.setPower(-1);
            delivery2.setPower(1);
        }
        else {
            intake.setPower(0);
            delivery1.setPower(0);
            delivery2.setPower(0);
        }
        if(gamepad1.right_trigger > .49)
            outtake.setPower(1);
        else if(gamepad1.left_trigger > .49)
            outtake.setPower(-1);
        else
            outtake.setPower(0);
        if(gamepad1.right_bumper)
            pully.setPosition(0);
        else
            pully.setPosition(1);
        if(gamepad1.x){
            duckLeft.setPosition(-.7);
            duckRight.setPosition(.7);}
        else{
            duckLeft.setPosition(.5);
            duckRight.setPosition(.5);
        }

        /*else{
            duckLeft.setPosition(.5);
        duckRight.setPosition(.5);}
        */

        


        telemetry.addData("Precision",togglePrecision);
        telemetry.addData("Reverse",reverse);
        //toggles precision mode if the right stick button is pressed


        //sets the factor multiplied to the power of the motors
        factor = togglePrecision ? .2 : 1; //the power is 1/5th of its normal value while in precision mode

        // Do not mess with this, if it works, it works
        double x = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double stickAngle = Math.atan2(direction ? -gamepad1.left_stick_y : gamepad1.left_stick_y, direction ? gamepad1.left_stick_x : -gamepad1.left_stick_x); // desired robot angle from the angle of stick
        double powerAngle = stickAngle - (Math.PI / 4); // conversion for correct power values
        double rightX = gamepad1.right_stick_x; // right stick x axis controls turning
        final double leftFrontPower = Range.clip(x * Math.cos(powerAngle)- rightX, -1.0, 1.0);
        final double leftRearPower = Range.clip(x * Math.sin(powerAngle) - rightX, -1.0, 1.0);
        final double rightFrontPower = Range.clip(x * Math.sin(powerAngle) + rightX, -1.0, 1.0);
        final double rightRearPower = Range.clip(x * Math.cos(powerAngle) +rightX, -1.0, 1.0);



        //Set the position of arm to counter clockwise/clockwise




        //neutral is .5, right trigger .5 to 1, left trigger is 0 to .5 What???


        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setPower(leftFrontPower * factor);
        leftBack.setPower(leftRearPower * factor);
        rightFront.setPower(rightFrontPower * factor);
        rightBack.setPower(rightRearPower * factor);

        //Incrementing the power by 0.0 EVERY TIME you call this function
        //Incrementing the power by 0.0 EVERY TIME you call this function


        //Reset the intake and transfer encoders


    }


}