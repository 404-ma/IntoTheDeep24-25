package org.firstinspires.ftc.teamcode.Helper.ReactiveState;


import java.lang.reflect.Method;

public class ReactiveState<T> {
    private Class<?> clazz;
    private T value;
    private String onChangeMethodName;
    private Object object;

    public ReactiveState(T value) {
        this.value = value;
    }

    public void setClass(Object obj) {
        this.object = obj;
        this.clazz = obj.getClass();
    }

    public void setOnChangeMethodName(String val) {
        this.onChangeMethodName = val;
    }

    public T get() {
        return this.value;
    }

    public ReactiveStateSetReturnType set(T value)  {
        this.value = value;

        return this.executeOnChangeEvent();
    }

    public ReactiveStateSetReturnType set(T value, boolean bypassReactivity)  {
        this.value = value;

        if (!(bypassReactivity)) {
            return this.executeOnChangeEvent();
        }

        return ReactiveStateSetReturnType.SUCCESS;
    }

    public ReactiveStateSetReturnType set() {
        return this.executeOnChangeEvent();
    }

    private ReactiveStateSetReturnType executeOnChangeEvent() {
        if ((this.clazz == null) || (this.onChangeMethodName == null)) {
            return ReactiveStateSetReturnType.NULL_PARAMS;
        }

        for (Method method : clazz.getMethods()) {
            if (method.getName().equals(this.onChangeMethodName)) {
                try {
                    method.invoke(this.object);
                } catch (Exception e) {
                    System.out.println(e.toString());
                }
                return ReactiveStateSetReturnType.INVOKE_ERROR;
            }
        }

        return ReactiveStateSetReturnType.SUCCESS;
    }
}