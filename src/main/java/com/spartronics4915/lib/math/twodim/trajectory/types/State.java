package com.spartronics4915.lib.math.twodim.trajectory.types;

import com.spartronics4915.lib.util.Interpolable;

public interface State<S> extends Interpolable<S> {
    double distance(S other);
}