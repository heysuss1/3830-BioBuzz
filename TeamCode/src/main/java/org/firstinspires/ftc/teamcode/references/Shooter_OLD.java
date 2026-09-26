package org.firstinspires.ftc.teamcode.references;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.controllers.PidfController;

private class Shooter_OLD {
    //made this private cuz i dont want it to mess with other code

    public static final ShootParams.Region[] shootRegions = {
            //Region 0: pitch 25 degrees, y = 3718.987 + 4.966x
            new ShootParams.Region(25.0, new double[][] {{2100,10}}),
            //Region 1:
            new ShootParams.Region(26.0, new double[][] {{1600,20}}),
            //Region 2:
            new ShootParams.Region(27.0, new double[][] {{2100,10}}),
            //Region 3:
            new ShootParams.Region(28.0, new double[][] {{1600,20}}),
            //Region 4:
            new ShootParams.Region(29.0, new double[][] {{1730,18}}),
            //Region 5:
            new ShootParams.Region(30.0, new double[][] {{1800,15}}),
            //Region 6:
            new ShootParams.Region(31.0, new double[][] {{2150,10}}),
            //Region 7:
            new ShootParams.Region(32.0, new double[][] {{2150,10}}),
            //Region 8:
            new ShootParams.Region(33.0, new double[][] {{2815,2}}),
            //Region 9:
            new ShootParams.Region(34.0, new double[][] {{2575,5}}),
            //Region 10:
            new ShootParams.Region(35.0, new double[][] {{2230,9}}),
            //Region 11:
            new ShootParams.Region(36.0, new double[][] {{3040,0.6}}),
            //Region 12: (far zone, probably make one more?)
            new ShootParams.Region(40.0, new double[][] {{3385,2}}),
            //From the middle of the zone (150~ distance, 41 pitch and 3775 rpm runs very nicce

    };

    public static final ShootParams shootParamsTable = new ShootParams()
            .addEntry("target_40in", 40.0, shootRegions[0], 2500)
            .addEntry("target_45in", 45.0, shootRegions[0], 2550)
            .addEntry("target_45in2", 45.01, shootRegions[1], 2500)
            .addEntry("target_50in", 50.0, shootRegions[1], 2600)
            .addEntry("target_50in2", 50.01, shootRegions[2], 2600)
            .addEntry("target_55in", 55.0, shootRegions[2], 2650)
            .addEntry("target_55in2", 55.01, shootRegions[3], 2700)
            .addEntry("target_60in", 60.0, shootRegions[3], 2800)
            //TODO: recheck targets 60 region 4 to 70 region 5
            .addEntry("target_60in2", 60.01, shootRegions[4], 2810)
            .addEntry("target_65in", 65.0, shootRegions[4], 2900)
            .addEntry("target_65in2", 65.01, shootRegions[5], 2775)
            .addEntry("target_70in", 70.0, shootRegions[5], 2850)
            .addEntry("target_70in2", 70.01, shootRegions[6], 2850)
            .addEntry("target_75in", 75.0, shootRegions[6], 2900)
            .addEntry("target_75in2", 75.01, shootRegions[7], 2900)
            .addEntry("target_80in", 80.0, shootRegions[7], 2950)
            .addEntry("target_80in2", 80.01, shootRegions[8], 2975)
            .addEntry("target_85in", 85.0, shootRegions[8], 2985)
            .addEntry("target_85in2", 85.01, shootRegions[9], 3000)
            .addEntry("target_90in", 90.0, shootRegions[9], 3025)
            .addEntry("target_90in2", 90.01, shootRegions[10], 3040)
            .addEntry("target_95in", 95.0, shootRegions[10], 3085)
            .addEntry("target_95in2", 95.01, shootRegions[11], 3097)
            .addEntry("target_100in", 100.0, shootRegions[11], 3100)
            .addEntry("target_140in2", 140.0, shootRegions[12], 3665)
            .addEntry("target_145in", 145.0, shootRegions[12], 3675)
    ;

    public static final class Params {

        //Shooter Params
        public static final double GOBUILDA_5000_CPR = 28.0;
        public static final double SHOOTER_MOTOR_GEAR_RATIO = 1;
        public static final double SHOOTER_TICKS_PER_REV = GOBUILDA_5000_CPR * SHOOTER_MOTOR_GEAR_RATIO;
//        public static final double SHOOTER_KP = 0.001, SHOOTER_KI = 0.01, SHOOTER_KD = 0, SHOOTER_KF = 0.0002, SHOOTER_I_ZONE = 150;
                public static final double SHOOTER_KP = 0.0012, SHOOTER_KI = 0.02, SHOOTER_KD = 0.00001, SHOOTER_KF = 0.0002, SHOOTER_I_ZONE = 150;

        public static final double SHOOTER_TOLERANCE_RPM = 100;

        //Pitch Params
        public static final double MIN_PITCH_DEGREES = 19;
        public static final double MAX_PITCH_DEGREES = 46;
        public static final double PITCH_GEAR_RATIO = .177 ;
        public static final double PITCH_ENCODER_ZERO_OFFSET = .49;
        public static final double PITCH_POSITION_OFFSET = MIN_PITCH_DEGREES;
        public static final double PITCH_DEGREES_PER_REV = 360 * PITCH_GEAR_RATIO;
        public static final double PITCH_TOLERANCE = 2;
        public static final double VEL_DROP_PITCH_SCALE = 0;

        //Turret Params
        public static final double TURRET_KP = .013, TURRET_KI = (.001), TURRET_KD = (0), TURRET_KF = (0.0001), TURRET_I_ZONE = (5);
        public static final double TURRET_GEAR_RATIO = 0.5;
        public static final double MIN_TURRET_DEGREES = -90;
        public static final double CROSSOVER_THRESHOLD = 0.5;
        public static final double MAX_TURRET_DEGREES = 90;
        public static final double TURRET_ENCODER_ZERO_OFFSET = 0;
        public static double TURRET_POSITION_OFFSET = 77;
        public static final double TURRET_DEGREES_PER_REV = -360 * TURRET_GEAR_RATIO;
        public static final double TURRET_TOLERANCE = 360;
    }



    private final DcMotorEx bottomShooterMotor;
    private final DcMotorEx topShooterMotor;
    private final Servo pitchServo;
    private final AnalogInput pitchEncoder;

    private final CRServo primaryTurretServo;
    private final CRServo secondaryTurretServo;
    private final AnalogInput turretEncoder;

    private final Telemetry telemetry;
    private final PidfController shooterController;
    private final PidfController turretController;

    public final Robot robot;

    public Double velocityTarget = null;

    double currentVelocityTarget;
    public Double pitchTarget = null;
    public Double turretTarget = null;

    public double currentVoltage = 0;
    public double prevVoltage = 0;
    private int crossovers = 0;

    public static boolean alwaysSetVelocity = false;
    public static boolean alwaysAimTurret = false;
    public static boolean alwaysAimPitch = false;
    private static boolean compensateForVelDropWithPitch = false;

    public Shooter_OLD(HardwareMap hwMap, Telemetry telemetry, Robot robot) {

        this.telemetry = telemetry;
        this.robot = robot;

        pitchServo = hwMap.get(Servo.class, "pitchServo");
        pitchServo.setDirection(Servo.Direction.REVERSE);
        pitchEncoder = hwMap.get(AnalogInput.class, "pitchEncoder");

        topShooterMotor = hwMap.get(DcMotorEx.class, "topShooterMotor");
        topShooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        topShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        topShooterMotor.setPower(0);

        bottomShooterMotor = hwMap.get(DcMotorEx.class, "bottomShooterMotor");
        bottomShooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        bottomShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bottomShooterMotor.setPower(0);

        primaryTurretServo = hwMap.get(CRServo.class, "primaryTurretServo");
        secondaryTurretServo = hwMap.get(CRServo.class, "secondaryTurretServo");
        secondaryTurretServo.setDirection(DcMotorSimple.Direction.FORWARD);
        turretEncoder = hwMap.get(AnalogInput.class, "turretEncoder");

        shooterController = new PidfController(Params.SHOOTER_KP, Params.SHOOTER_KI, Params.SHOOTER_KD, Params.SHOOTER_KF, Params.SHOOTER_I_ZONE);
        turretController = new PidfController(Params.TURRET_KP, Params.TURRET_KI, Params.TURRET_KD, Params.TURRET_KF, Params.TURRET_I_ZONE);
    }

    public PidfController getShooterController() {
        return shooterController;
    }

    public PidfController getTurretController() {
        return turretController;
    }

    public CRServo getPrimaryTurretServo(){
        return primaryTurretServo;
    }
    public CRServo getSecondaryTurretServo(){
        return secondaryTurretServo;
    }
    public DcMotorEx getShooterMotor() {
        return topShooterMotor;
    }

    public CRServo getTurretServo() {
        return primaryTurretServo;
    }

    public Servo getPitchServo() {
        return pitchServo;
    }

    public Double getVelocityTarget() {
        return velocityTarget;
    }

    public double getPitchTarget(){
        return pitchTarget;
    }

    public double getTurretTarget(){
        return turretTarget;
    }

    public double getRawPitchPos() {
        return 1- (pitchEncoder.getVoltage()/pitchEncoder.getMaxVoltage());
    }

    public double getTurretRawPose() {
        return turretEncoder.getVoltage()/turretEncoder.getMaxVoltage();
    }

    public Double getVelocityRPM() {
        return (Math.abs(topShooterMotor.getVelocity()) * 60)/ Params.SHOOTER_TICKS_PER_REV;
    }

    public double getPitchDegrees() {
        double rawPitchPos = (pitchEncoder.getVoltage() / pitchEncoder.getMaxVoltage());
        return rawPitchToDegrees(rawPitchPos);
    }

    public double getTurretDegrees() {
        double rawTurretPos = getTurretRawPose() + 0.987 * crossovers;
        double encoderOffset = (rawTurretPos - Params.TURRET_ENCODER_ZERO_OFFSET);
        double unconverted = (encoderOffset * Params.TURRET_DEGREES_PER_REV) + Params.TURRET_POSITION_OFFSET;
        return modularConversion(unconverted);
    }

    public int getCrossovers() {
        return crossovers;
    }

    public double rawPitchToDegrees(double rawPitchPos) {
        double encoderOffset = (rawPitchPos - Shooter.Params.PITCH_ENCODER_ZERO_OFFSET);
        return (encoderOffset * Params.PITCH_DEGREES_PER_REV) + Shooter.Params.PITCH_POSITION_OFFSET;
    }

    public double degreesToRawPitch(double degrees) {
        double clippedDegrees = Range.clip(degrees, Params.MIN_PITCH_DEGREES, Params.MAX_PITCH_DEGREES);
        double pitchZeroOffset = (clippedDegrees - Params.PITCH_POSITION_OFFSET) / (Params.PITCH_DEGREES_PER_REV);
        return pitchZeroOffset + Params.PITCH_ENCODER_ZERO_OFFSET;
    }

    public void setVelocityTarget(Double velocityTargetRPM) {
        this.velocityTarget = velocityTargetRPM;
    }

    public void setPitchDegrees(Double targetPitchDegrees) {
        pitchTarget = degreesToRawPitch(targetPitchDegrees);
    }

    public void setTurretDegrees(Double targetDegrees) {
        if (targetDegrees == null){
            turretTarget = null;
            return;
        } else if (targetDegrees == 0){
            turretTarget = 0.0;
            return;
        }

        double turretTargetRaw = (targetDegrees - Math.toDegrees(robot.follower.getHeading())) * -1;
        double turretTargetModulo = robot.shooter.modularConversion(turretTargetRaw);

        turretTarget = Range.clip(turretTargetModulo, -90, 90);
        telemetry.addData("Turret Target (actual)", turretTargetRaw);
        //TODO: Why are you only clipping to 160 and not 180? This seems like a band-aid.
    }

    public double modularConversion(double n) {
        return (Math.floorMod((long)(n + 180), 360) - 180);
    } //ts needs to work with decimals appparently. dunno why we didnt js use radians ig

    public void countCrossovers() {

        prevVoltage = currentVoltage;
        currentVoltage = getTurretRawPose();

        if (Math.abs(currentVoltage - prevVoltage) > Params.CROSSOVER_THRESHOLD) {
            if (currentVoltage > prevVoltage) {
                crossovers--;
            }
            else if (currentVoltage < prevVoltage) {
                crossovers++;
            }
        }
    }

    public double velDropCompensationWithPitch() {
        //Note: This would be added to Degrees before being converted to raw pitch
        return getShooterController().getError() * Params.VEL_DROP_PITCH_SCALE;
    }

    public void stopShooterMotor() {
        if (velocityTarget != null) {
            velocityTarget = null;
            topShooterMotor.setPower(0);
            bottomShooterMotor.setPower(topShooterMotor.getPower());
        }
    }

    public void stopPitchServo() {
        if (pitchTarget != null) {
            pitchTarget = null;
        }
    }

    public void stopTurret() {
        if (turretTarget != null) {
            turretTarget = null;
            primaryTurretServo.setPower(0);
            secondaryTurretServo.setPower(0);
        };
    }

    public void stopShooter() {
        stopShooterMotor();
        stopPitchServo();
        stopTurret();
    }

    public boolean isFlywheelOnTarget(double tolerance) {
        boolean isOnTarget = false;
        if (velocityTarget != null) {
            isOnTarget = (Math.abs(velocityTarget - getVelocityRPM()) <= tolerance);
        }
        return isOnTarget;
    }

    public boolean isPitchOnTarget(double tolerance) {
        boolean isOnTarget = false;
        if (pitchTarget != null) {
            isOnTarget = Math.abs(rawPitchToDegrees(pitchTarget) - getPitchDegrees()) <= tolerance;
        }
        return isOnTarget;
    }

    public boolean isTurretOnTarget(double tolerance) {
        boolean isOnTarget = false;
        if (turretTarget != null) {
            isOnTarget = Math.abs(turretTarget - getTurretDegrees()) <= tolerance;
        }
        return isOnTarget;
    }

    public boolean isShooterReady(double flywheelTolerance, double turretTolerance, double pitchTolerance) {
        return (velocityTarget == null || isFlywheelOnTarget(flywheelTolerance));
//                && (turretTarget == null || isTurretOnTarget(turretTolerance))
//                && (pitchTarget == null || isPitchOnTarget(pitchTolerance));
    }


    public boolean isShooterReady() {
        return isShooterReady(Params.SHOOTER_TOLERANCE_RPM, Params.TURRET_TOLERANCE, Params.PITCH_TOLERANCE);
    }

    public void flywheelTask() {
        double output = 0;
        if (velocityTarget != null) {
            double currentVelocity = getVelocityRPM();
            output = shooterController.calculate(velocityTarget, currentVelocity);
            topShooterMotor.setPower(output);
            bottomShooterMotor.setPower(topShooterMotor.getPower());
        }
        else
        {
            //Have the shooter idle down to zero instead of wasting power.
            topShooterMotor.setPower(0);
            bottomShooterMotor.setPower(topShooterMotor.getPower());
        }
        if (!Robot.inComp) {
            telemetry.addLine("\nFlywheel Info:");
            telemetry.addData("Target Velocity (rpm)", velocityTarget );
            telemetry.addData("Current Velocity (rpm)", getVelocityRPM());
            telemetry.addData( "Power", output);
            telemetry.addData("Error", shooterController.getError());
        }
    }

    public void pitchTask() {
        if (pitchTarget != null) {
            pitchServo.setPosition(1-(1.160*pitchTarget-0.088));
        }
        if (!Robot.inComp) {
            telemetry.addLine("\nPitch Info:");
            telemetry.addData("Pitch Target", pitchTarget);
            telemetry.addData("Current Pitch Degrees", getPitchDegrees());
        }
    }

    public void turretTask() {
        double output = 0;
        countCrossovers();

        if (turretTarget != null) {
            double currentPosition = getTurretDegrees();
            output = turretController.calculate(turretTarget, currentPosition) ;
//            if (output > 0) output += 0.0004;
//            if (output < 0) output -= 0.0004;
            primaryTurretServo.setPower(output);
            secondaryTurretServo.setPower(output);

        }

        if (!Robot.inComp) {
            telemetry.addLine("\nTurret Info:");
            telemetry.addData("Turret Target", turretTarget);
            telemetry.addData("Current Turret Degrees", getTurretDegrees());
            telemetry.addData( "Power", output);
            telemetry.addData("Error", turretController.getError());
            telemetry.update();
        }
    }

    public void shooterTask() {
        flywheelTask();
        pitchTask();
        turretTask();

        Robot.AimInfo aimInfo = robot.getAimInfo();
        //Find rpm and pitch from the shootparams table based on distance to goal.
        ShootParams.Entry shootParams = Shooter.shootParamsTable.get(aimInfo.getDistanceToGoal());

        if(compensateForVelDropWithPitch) {
            double pitchCompensation = velDropCompensationWithPitch();
            if(alwaysAimPitch) {
                setPitchDegrees(shootParams.region.tiltAngle + pitchCompensation);
            }
            else {
                pitchTarget = pitchTarget != null ? pitchTarget + pitchCompensation : Params.MIN_PITCH_DEGREES + pitchCompensation;
            }
        }
        else {
            if(alwaysAimPitch) {
                setPitchDegrees(shootParams.region.tiltAngle);
            }
        }

        if (alwaysAimTurret) {
            setTurretDegrees(aimInfo.getAngleToGoal());
            telemetry.addData("Target from shooter class",aimInfo.getAngleToGoal() );
        }

        if (alwaysSetVelocity) {
            if (robot.isInRevUpZone()){
                velocityTarget = shootParams.outputs[0];
            } else {
                stopShooterMotor();
            }
        }
        //Always setting pitch to the calculated angle from the shootparams table.
    }
}
