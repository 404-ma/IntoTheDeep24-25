package org.firstinspires.ftc.teamcode.Helper;

public interface Subscriber<T> {
    public T onMessage(Object ... keys);
}