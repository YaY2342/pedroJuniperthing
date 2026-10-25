package org.firstinspires.ftc.teamcode.juniper.statemachine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class StateMachine {
    private List<State> states;
    private State cur;
    private Map<State, Integer> indexMap;
    private int index;
    private boolean isOn;

    public StateMachine(List<State> states) {
        this.states = states;
        this.index = 0;
        isOn = false;
    }

    public StateMachine() {
        this.states = new ArrayList<>();
        this.index = 0;
        isOn = false;
    }

    public StateMachine addState (State state) {
        states.add(state);
        return this;
    }

    public void start(int startIndex) {
        if (!states.isEmpty()) {
            indexMap = new HashMap<>(states.size());

            for (int i = 0; i < states.size(); i++) {
                indexMap.put(states.get(i), i);
            }

            index = startIndex;
            isOn = true;
            cur = states.get(startIndex);
            cur.start();
        }
    }

    public void start() {
        start(0);
    }

    public void run() {
        if (!isOn) return;

        cur.run();

        if (cur.isFinished()) {
            State nState = cur.getNextState();
            if (nState == null) isOn = false;
            else {
                cur = nState;
                index = indexMap.get(cur);
                cur.start();
            }
        }
    }

    public String currentState() {
        return cur.getName();
    }

    public boolean isFinished() {
        return !isOn;
    }
}