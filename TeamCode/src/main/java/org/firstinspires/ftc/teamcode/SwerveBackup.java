package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class SwerveBackup extends OpMode {
    final double L = 0.98;
    final double W = 1.00;

    DcMotorEx frontLeftMotor;
    AnalogInput frontLeftAnalog;
    CRServo frontLeftServo;

    DcMotorEx frontRightMotor;
    AnalogInput frontRightAnalog;
    CRServo frontRightServo;

    DcMotorEx backLeftMotor;
    AnalogInput backLeftAnalog;
    CRServo backLeftServo;

    DcMotorEx backRightMotor;
    AnalogInput backRightAnalog;
    CRServo backRightServo;

    PIDController flPID;
    PIDController frPID;

    PIDController rlPID;
    PIDController rrPID;

    ElapsedTime angleTimer = new ElapsedTime();

//    SparkFunOTOS otos;



    double FLkP = 0.002;
    double FLkI = 0.0;
    double FLkD = 0.0001;
    double FLkF = 0.0;

    double FRkP = 0.002;
    double FRkI = 0.0;
    double FRkD = 0.0001;
    double FRkF = 0.0;

    double RLkP = 0.002;
    double RLkI = 0.0;
    double RLkD = 0.0001;
    double RLkF = 0.0;

    double RRkP = 0.002;
    double RRkI = 0.0;
    double RRkD = 0.0001;
    double RRkF = 0.0;

    double inputAngle = 0;
    double turnSpeedDeg = 60;

    double FL_OFFSET = 6; // Test these offsets LMAO
    double FR_OFFSET = -6.4;
    double BL_OFFSET = 19;
    double BR_OFFSET = -13.4;


    final double GEARBOX_RATIO = 1 / (36.0f / 24.0f);

    double minServoPower = 0.03;

    double lastTargetFR = 0, lastTargetFL = 0, lastTargetRL = 0, lastTargetRR = 0;

    double speed = 1;

//    boolean wasFlippedFL, wasFlippedFR, wasFlippedBL, wasFlippedBR = false;

    double flSpeed, frSpeed, blSpeed, brSpeed = 0;

    double angle_fl, angle_fr, angle_rl, angle_rr = 0;

    double initFR = 0;
    double initFL = 0;
    double initRL = 0;
    double initRR = 0;

    ElapsedTime dx = new ElapsedTime();

    ElapsedTime waitForCalibration = new ElapsedTime();

    double tgtAngle = 0;

    double speedMul = 1;

    @Override
    public void init()
    {
        frontLeftMotor  = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontLeftServo  = hardwareMap.get(CRServo.class,     "frontLeftServo");
        frontLeftAnalog = hardwareMap.get(AnalogInput.class, "frontLeftAnalog");

        frontRightMotor  = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        frontRightServo  = hardwareMap.get(CRServo.class,     "frontRightServo");
        frontRightAnalog = hardwareMap.get(AnalogInput.class, "frontRightAnalog");

        backLeftMotor  = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backLeftServo  = hardwareMap.get(CRServo.class,     "backLeftServo");
        backLeftAnalog = hardwareMap.get(AnalogInput.class, "backLeftAnalog");

        backRightMotor  = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        backRightServo  = hardwareMap.get(CRServo.class,     "backRightServo");
        backRightAnalog = hardwareMap.get(AnalogInput.class, "backRightAnalog");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);


//        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
//        otos.calibrateImu();

        flPID = new PIDController(FLkP, FLkI, FLkD, FLkF);
        frPID = new PIDController(FRkP, FRkI, FRkD, FRkF);
        rlPID = new PIDController(RLkP, RLkI, RLkD, RLkF);
        rrPID = new PIDController(RRkP, RRkI, RRkD, RRkF);
    }

    boolean canSetInit = true;

    @Override
    public void loop()
    {
        if (waitForCalibration.seconds() > 0.5 && canSetInit)
        {
            initFL = getAngle(frontLeftAnalog, FL_OFFSET);
            initFR = getAngle(frontRightAnalog, FR_OFFSET);
            initRL = getAngle(backLeftAnalog, BL_OFFSET);
            initRR = getAngle(backRightAnalog, BR_OFFSET);

            canSetInit = false;
        }
        double leftStickX = gamepad1.left_stick_x;
        double leftStickY = -gamepad1.left_stick_y;

        double rightStickX = gamepad1.right_stick_x;
        boolean reset = gamepad1.options;

        dx.reset();

        swerveDrive(leftStickY, leftStickX, rightStickX, reset);

        telemetry.addData("servo FL:", normalizeAngle(getAngle(frontLeftAnalog, FL_OFFSET)));
        telemetry.addData("servo FR :", normalizeAngle(getAngle(frontRightAnalog, FR_OFFSET)));
        telemetry.addData("servo BL :", normalizeAngle(getAngle(backLeftAnalog, BL_OFFSET)));
        telemetry.addData("servo BR :", normalizeAngle(getAngle(backRightAnalog, BR_OFFSET)));

        telemetry.addData("FL Speed", flSpeed);
        telemetry.addData("FR Speed", frSpeed);
        telemetry.addData("RL Speed", blSpeed);
        telemetry.addData("RR Speed", brSpeed);

        telemetry.addData("FL Angle", angle_fl);
        telemetry.addData("FR Angle", angle_fr);
        telemetry.addData("RL Angle", angle_rl);
        telemetry.addData("RR Angle", angle_rr);
//        telemetry.addData("FL Target", "%.1f°", lastTargetFL);;
        telemetry.update();
    }

    public void swerveDrive(double y_cmd_field, double x_cmd_field, double turn_cmd, boolean reset) {
        double heading_rad = Math.toRadians(0);

        double x_cmd = x_cmd_field * Math.cos(heading_rad) + y_cmd_field * Math.sin(heading_rad);
        double y_cmd = -x_cmd_field * Math.sin(heading_rad) + y_cmd_field * Math.cos(heading_rad);

        if (Math.hypot(x_cmd, y_cmd) < 0.05 && Math.abs(turn_cmd) < 0.05) {
            stopDrive();
            return;
        }

        if (y_cmd_field < -0.2)
        {
            tgtAngle = 0;
            speedMul = -1;
        }
        else if (y_cmd_field > 0.2)
        {
            speedMul = 1;
            tgtAngle = 0;
        }
        else if (x_cmd_field > 0.2)
        {
            speedMul = 1;
            tgtAngle = 90;
        }
        else if (x_cmd_field < -0.2)
        {
            speedMul = -1;
            tgtAngle = 90;
        }
        else if (turn_cmd > 0.2)
        {
            frSpeed = speed;
            flSpeed = speed;
            blSpeed = -speed;
            brSpeed = -speed;

            frontRightMotor.setPower(frSpeed);
            frontLeftMotor.setPower(flSpeed);
            backLeftMotor.setPower(blSpeed);
            backRightMotor.setPower(brSpeed);

            // fl fr rl rr
            runPID(45, 135, 135, 45);

            lastTargetFL = tgtAngle;
            lastTargetFR = tgtAngle;
            lastTargetRL = tgtAngle;
            lastTargetRR = tgtAngle;
            return;
        }
        else if (turn_cmd < -0.2)
        {
            frSpeed = -speed;
            flSpeed = -speed;
            blSpeed = speed;
            brSpeed = speed;

            frontRightMotor.setPower(frSpeed);
            frontLeftMotor.setPower(flSpeed);
            backLeftMotor.setPower(blSpeed);
            backRightMotor.setPower(brSpeed);

            // fl fr rl rr
            runPID(45, 135, 135, 45);

            lastTargetFL = tgtAngle;
            lastTargetFR = tgtAngle;
            lastTargetRL = tgtAngle;
            lastTargetRR = tgtAngle;
            return;
        }

        frSpeed = speed * speedMul;
        flSpeed = speed * speedMul;
        blSpeed = speed * speedMul;
        brSpeed = speed * speedMul;

        frontRightMotor.setPower(frSpeed);
        frontLeftMotor.setPower(flSpeed);
        backLeftMotor.setPower(blSpeed);
        backRightMotor.setPower(brSpeed);

        runPID(tgtAngle, tgtAngle, tgtAngle, tgtAngle);

        lastTargetFL = tgtAngle;
        lastTargetFR = tgtAngle;
        lastTargetRL = tgtAngle;
        lastTargetRR = tgtAngle;
    }

    private void stopDrive() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        runPID(lastTargetFL,lastTargetFR,lastTargetRL,lastTargetRR);
    }

    private void runPID(double tFL ,double tFR, double tRL, double tRR) {
        double dt = angleTimer.seconds();
        if (dt < 0.001) dt = 0.001;

        double servoTargetFL = tFL;
        double servoTargetFR = tFR;
        double servoTargetRL = tRL;
        double servoTargetRR = tRR;

        double currentServoAngleFL = normalizeAngle(getAngle(frontLeftAnalog, FL_OFFSET));
        double currentServoAngleFR = normalizeAngle(getAngle(frontRightAnalog, FR_OFFSET));
        double currentServoAngleRL = normalizeAngle(getAngle(backLeftAnalog, BL_OFFSET));
        double currentServoAngleRR = normalizeAngle(getAngle(backRightAnalog, BR_OFFSET));

        double powerFL = flPID.calculate(servoTargetFL, currentServoAngleFL);
        double powerFR = frPID.calculate(servoTargetFR, currentServoAngleFR);
        double powerRL = rlPID.calculate(servoTargetRL, currentServoAngleRL);
        double powerRR = rrPID.calculate(servoTargetRR, currentServoAngleRR);

        angleTimer.reset();

        frontLeftServo.setPower(powerFL);
        frontRightServo.setPower(powerFR);
        backLeftServo.setPower(powerRL);
        backRightServo.setPower(powerRR);

        lastTargetFL = tFL;
        lastTargetFR = tFR;
        lastTargetRL = tRL;
        lastTargetRR = tRR;
    }

    private double getAngle(AnalogInput sensor, double offset)
    {
        double rawAngle = (sensor.getVoltage() / 3.3) * 360.0;

        double adjustedAngle = (rawAngle/GEARBOX_RATIO) - offset; // Coordinate missmatch between servo and wheel space!

        // angle between 0 and 360
        if (adjustedAngle < 0)
        {
            adjustedAngle *= -1;
            adjustedAngle = 360 - adjustedAngle;
        }

        return adjustedAngle;
    }

    private double normalizeAngle(double angle)
    {
        angle = (angle + 180.0) % 360.0;
        if (angle < 0) angle += 360.0;
        return angle - 180; // no longer doing the shift toward - values, so this should also be 0-360
    }

    public class PIDController {
        private double kP, kI, kD, kF;
        private double lastError = 0;
        private ElapsedTime timer = new ElapsedTime();

        public PIDController(double kP, double kI, double kD, double kF) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
            timer.reset();
        }

        public double calculate(double target, double current) {
            double error = normalizeAngle(target - current);
            double dt = timer.seconds();

            if (dt < 0.001) dt = 0.001;

            double pTerm = error * kP;
            double derivative = (error - lastError) / dt;
            double dTerm = derivative * kD;

            // PLEASE SPEED I NEED THIS!!!
            // MY FEED IS KINDA FORWARDLESS!!!
//            double fTerm = targetVel * kF;

            lastError = error;

            double output = pTerm + dTerm;

            if (Math.abs(error) < 15.0)
                return Math.signum(error) * minServoPower;

            output += Math.signum(output) * minServoPower;

            timer.reset();

            return Range.clip(output, -1.0, 1.0);
        }
    }
}