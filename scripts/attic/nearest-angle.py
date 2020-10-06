def nearest_angle(th_curr: float, th_dest: float) -> float:
    """
    >>> nearest_angle(10, 350)
    -10.
    >>> nearest_angle(350, 720)
    360.
    >>> nearest_angle(370, 1080 + 20)
    380.
    >>> for th_curr in range(-1000, 1000, 5):
    ...     for th_dest in range(-1000, 1000, 7):
    ...         assert abs(nearest_angle(th_curr, th_dest) - th_curr) <= 180,\
    ...             f"error: got: {nearest_angle(th_curr, th_dest)}. {th_curr}"
    """
    from math import fmod
    error = th_dest - th_curr
    if -180 < error < 180:
        return th_dest
    else:
        err_0_360 = fmod(error + 360*(abs(error)//360+1), 360)
        if err_0_360 > 180:
            return th_curr + err_0_360 - 360
        else:
            return th_curr + err_0_360
