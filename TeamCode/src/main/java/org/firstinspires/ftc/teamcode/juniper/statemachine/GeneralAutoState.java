package org.firstinspires.ftc.teamcode.juniper.statemachine;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

public abstract class GeneralAutoState implements State {

    private BaseState path, action;

    private Runnable duringPath, beforePath, afterPath;

    private List<Transition> transitions;
    private boolean hasTransitions = false;

    private String name;
    private State nextState;
    private boolean isRunning = false;

    public GeneralAutoState(String name) {
        isRunning = false;
        this.name = name;
        path = new BaseState(name);
        action = new BaseState(name + "A");
        path.setFallbackState(action);
        transitions = new ArrayList<>();

        transitions.add(new Transition(this::pathDone, nextState));
    }

    abstract public GeneralAutoState setPath();
    abstract public boolean pathDone();
    abstract public void followPath(); // if its done return true

    public GeneralAutoState prePath(Runnable action) {
        beforePath = action;
        return this;
    }
    public GeneralAutoState postPath(Runnable action) {
        afterPath = action;
        return this;
    }
    public GeneralAutoState duringPath(Runnable action) {
        duringPath = action;
        return this;
    }

    public GeneralAutoState addTransition(Transition t) {
        if (!hasTransitions) {
            transitions.clear();
        }
        transitions.add(t);
        hasTransitions = true;
        return this;
    }

    public GeneralAutoState maxPathTime(long millis) {
        path.setMaxTimeMS(millis);
        return this;
    }

    public GeneralAutoState setAction (Runnable function, BooleanSupplier condition) {
        action = new BaseState(name + "A");
        action.setDuring(function);
        action.addTransition(new Transition(condition, () -> null));
        return this;
    }
    public GeneralAutoState setAction (BaseState function) {
        action = function;
        return this;
    }

    @Override
    public void start() {
        isRunning = true;

        path.setEntry(() -> {
            if (beforePath != null) beforePath.run();
            followPath();
        });

        if (action != null) {
            path.setExit(() -> {
                if (afterPath != null) afterPath.run();
                action.start();
            });
        } else path.setExit(afterPath);

        path.start();
    }

    @Override
    public void run() {
        if (!isRunning) return;

        if (!path.isFinished()) {
            path.run();
            if (duringPath != null) duringPath.run();
        } else {
            for (Transition t : transitions) {
                t.run();
                if (t.isFinished()) {
                    nextState = t.getNextState();
                    isRunning = false;
                    return;
                }
            }
            if (action != null && !action.transitionsAreTrue()) {
                action.run();
            } else {
                isRunning = false;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return !isRunning;
    }

    @Override
    public String getName() {
        return name;
    }

    public GeneralAutoState setNextState(State nextState) {
        this.nextState = nextState;
        return this;
    }
    @Override
    public State getNextState() {
        return nextState;
    }
}