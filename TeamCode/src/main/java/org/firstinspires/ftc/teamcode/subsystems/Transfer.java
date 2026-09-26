package org.firstinspires.ftc.teamcode.subsystems;

import androidx.appcompat.widget.WithHint;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Transfer {
    private DcMotor intake;
    private DcMotor transfer;
    private Servo raiseIntakeL;
    private Servo raiseIntakeR;
    private double intakeLeftUpPosition;
    private double intakeRightUpPosition;
    private double intakeLeftDownPosition;
    private double intakeRightDownPosition;
    // intake, uptake speed constants
    TransferStates transferState;
    final double INTAKE_SPEED = 0.8;
    final double UPTAKE_BLOCK_SPEED = -0.3;

    public Transfer(HardwareMap hwMap){
        intake = hwMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //stop and reset encoders?
        intake.setPower(0);

        transfer = hwMap.get(DcMotor.class, "intake");
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setPower(0);

        raiseIntakeR = hwMap.get(Servo.class, "raiseIntakeR");
        raiseIntakeL = hwMap.get(Servo.class, "raiseIntakeL");


    }
    enum TransferStates{
        INTAKE, UPTAKE, HOLD, EMPTY
    }

    public TransferStates getTransferState(){
        return transferState;
    }
    public void setTransferState(TransferStates transferState){
        this.transferState = transferState;
    }
    public void raiseIntake(){
        raiseIntakeL.setPosition(intakeLeftUpPosition);
        raiseIntakeR.setPosition(intakeRightUpPosition);
    }
    public void lowerIntake(){
        raiseIntakeL.setPosition(intakeLeftDownPosition);
        raiseIntakeR.setPosition(intakeRightDownPosition);
    }

    public void transferUpdate(){
        switch(transferState){
            case HOLD:
                intake.setPower(0);
                transfer.setPower(0);
                raiseIntake();
                break;
            case EMPTY:
                raiseIntake();
                intake.setPower(-INTAKE_SPEED);
                transfer.setPower(-INTAKE_SPEED);
            case INTAKE:
                lowerIntake();
                intake.setPower(INTAKE_SPEED);
                transfer.setPower(UPTAKE_BLOCK_SPEED);
            case UPTAKE:
                 raiseIntake();
                 intake.setPower(INTAKE_SPEED);
                 transfer.setPower(INTAKE_SPEED);
        }
    }
    public void setHoldMode(){
        setTransferState(TransferStates.HOLD);
    }
    public void setIntakeMode(){
        setTransferState(TransferStates.INTAKE);

    }
    public void setEmptyMode(){
        setTransferState(TransferStates.EMPTY);
    }
    public void setUptakeMode(){
        setTransferState(TransferStates.UPTAKE);
    }

}
