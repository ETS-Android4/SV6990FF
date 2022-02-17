package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class BlueAutoCarry extends LinearOpMode {
    private DcMotorEx leftFront, leftBack, rightFront, rightBack, intake, outtake, delivery1, delivery2;
    private Servo duckRight, pully, duckleft;
    private DcMotorEx[] motors;
    private final double TPI = 33.5625;
    private ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        drive(1, 300);

        sleep(1);

        raisePully(2000);

        deposit(500);

        raiseBasket(500);

        dropPully(930);

        sleep(1);

        drive(-1, 110);

        sleep(1);

        strafe(-1, 1300);

        strafe(-.25, 1600);

        sleep(1);

        dropDuck(4100);      // Duck spinner

        //heading to warehouse

        sleep(1);

        strafe(1, 300);

        sleep(50);

        turn(-1, 250);

        sleep(100);

        strafe(-1,400);

        sleep(100);

        strafe(1, 1430);

        sleep(1);

        drive(.5, 950);

        sleep(1);

        drive(-1, 800);

        drive(-.5, 670);

        strafe(-1, 1200);

        drive(-.75, 825); //enter to warehouse

        sleep(50);

        strafe(-1, 750);

        sleep(50);

        intake.setPower(-1);
        delivery1.setPower(1);
        delivery2.setPower(-1);

        drive(-.5, 810); // start cycle

        drive(0, 100);

        drive(.5, 300);

        strafe(-1, 1200);

        drive(0, 50);

        strafe(1, 850);

        drive(0, 75);

        drive(1, 975);

        turn(1,250);

        raisePully(2000);

        deposit(500);

        raiseBasket(500);

        dropPully(950);

        turn(-1, 150);

        drive(-1, 1400);

    }

    public void initialize() {
        leftFront = (DcMotorEx) hardwareMap.dcMotor.get("FL");
        leftBack = (DcMotorEx) hardwareMap.dcMotor.get("BL");
        rightFront = (DcMotorEx) hardwareMap.dcMotor.get("FR");
        rightBack = (DcMotorEx) hardwareMap.dcMotor.get("BR");
        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        delivery1 = (DcMotorEx) hardwareMap.dcMotor.get("delivery1");
        delivery2 = (DcMotorEx) hardwareMap.dcMotor.get("delivery2");
        outtake = (DcMotorEx) hardwareMap.dcMotor.get("outtake");
        duckRight = (Servo) hardwareMap.get("duckRight");
        pully = (Servo) hardwareMap.get("pully");
        duckleft = (Servo) hardwareMap.get("duckLeft");

        motors = new DcMotorEx[]{leftFront, leftBack, rightFront, rightBack};

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        delivery1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        delivery2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();
    }

    public void drive(double power, int time) throws InterruptedException {
        timer = new ElapsedTime();
        for (DcMotorEx motor : motors) {
            motor.setPower(power);
        }
        while (timer.milliseconds() <= time) {
            if (!opModeIsActive()) {
                throw new InterruptedException();
            }
        }
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
    }

    public void strafe(double direction, int time) throws InterruptedException {
        timer = new ElapsedTime();
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        for (DcMotorEx motor : motors) {
            motor.setPower(1 * direction);
        }
        while (timer.milliseconds() <= time) {
            if (!opModeIsActive()) {
                throw new InterruptedException();
            }
        }
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void turn(int power, int time) throws InterruptedException {
        timer = new ElapsedTime();
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        for (DcMotorEx motor : motors) {
            motor.setPower(power);
        }
        while (timer.milliseconds() <= time) {
            if (!opModeIsActive()) {
                throw new InterruptedException();
            }
        }
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
    }


    public void intakeBox() throws InterruptedException {
        timer = new ElapsedTime();

        intake.setPower(-1);
        delivery1.setPower(1);
        delivery2.setPower(-1);

            if (!opModeIsActive()) {
                throw new InterruptedException();
            }

        intake.setPower(0);
        delivery1.setPower(0);
        delivery2.setPower(0);

    }

    public void dropDuck(int time) throws InterruptedException {
        duckRight.setPosition(.60);

        timer = new ElapsedTime();
        while (timer.milliseconds() <= time) {
            if (!opModeIsActive()) {
                throw new InterruptedException();
            }
        }
        duckRight.setPosition(.5);
    }

    public void raisePully(int time) throws InterruptedException {
        timer = new ElapsedTime();
        outtake.setPower(1);

        while (timer.milliseconds() <= time)
            if (!opModeIsActive()) {
                throw new InterruptedException();
            }
       /* timer = new ElapsedTime();
        pully.setPosition(0);
        while (timer.milliseconds() <= time) {
            if (!opModeIsActive()) {
                throw new InterruptedException();
            }
*/

    }


    public void dropPully(int time) throws InterruptedException {

       /* timer = new ElapsedTime();
        pully.setPosition(1);
        while (timer.milliseconds() <= time)
            if (!opModeIsActive()) {
                throw new InterruptedException();
            }
            */

        timer = new ElapsedTime();
        outtake.setPower(-1);
        while (timer.milliseconds() <= time) {
            if (!opModeIsActive()) {
                throw new InterruptedException();
            }


        }
        outtake.setPower(0);
    }

    public void deposit(int time) throws InterruptedException {
        timer = new ElapsedTime();
        pully.setPosition(0);

        while (timer.milliseconds() <= time)
            if (!opModeIsActive()) {
                throw new InterruptedException();
            }

    }

    public void raiseBasket(int time) throws InterruptedException {
        timer = new ElapsedTime();
        pully.setPosition(1);

        while (timer.milliseconds() <= time)
            if (!opModeIsActive()) {
                throw new InterruptedException();
            }
    }
}
