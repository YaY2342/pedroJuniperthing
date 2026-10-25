package org.firstinspires.ftc.teamcode.juniper.statemachine;

public interface State {
    void start();
    void run();
    boolean isFinished();
    String getName();
    State getNextState();
}
