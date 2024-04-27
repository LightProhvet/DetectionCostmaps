from collections import deque
import warnings


class CircularTimeBuffer(deque):

    def _sec_check(self, value):
        if value != int(value) and not self._ns_settable:
            warnings.warn("Integer values are used for seconds. We are discarding the floating point values "
                          "If you wish to set nanosecond values with floating point numbers use "
                          "kwarg ns_settable=True on declaration", UserWarning)

    @staticmethod
    def _ns_check(value):
        if value != int(value):
            warnings.warn(
                "Integer values are used for nanoseconds. We are discarding the floating point values.",
                UserWarning
            )

    # The default buffer with period 0 should work exactly like a deque - only a bit slower maybe
    def __init__(self, maxlen=0, period=0, period_ns=0, continuous=False, ns_settable=False, null_settable=False):
        super().__init__(maxlen=maxlen)
        # static attributes

        # A value to decide whether to discard floating point second inputs
        # if False: when self.last_update = 1.25 ==> self.last_update_sec = 1 and self.last_update_ns = 0
        # if True: when self.last_update = 1.25 ==> self.last_update_sec = 1 and self.last_update_ns = 0.25*1e9
        self._ns_settable = ns_settable
        # a value to decide whether excessive dt we reset the last update to 0 (False) or to the excess (True)
        self._continuous = continuous
        # a value to decide whether allow inserting empty values (None, False, 0, [], {} and other bool(x)=False)
        self._null_settable = null_settable
        # buffer minimal update period in seconds and nanoseconds
        self._time_limit_sec = int(period)
        if period != int(period) and self._ns_settable and not period_ns:
            self.last_update_ns = int((period - int(period)) * 1e9)  # int conversion for ridiculous length floats
        else:
            if period_ns != int(period_ns):
                raise TypeError("an integer is required for ns_period")
            if period != int(period):
                raise UserWarning(
                    "Integer values are used for period. "
                    "If you wish to set nanosecond values with floating point numbers use kwarg ns_settable=True"
                )
            self._time_limit_ns = period_ns
        # dynamic attributes - last update time
        self.last_update_sec = 0
        self.last_update_ns = 0

    @property
    def period(self):
        return self._time_limit_sec + self._time_limit_ns / 1e9

    @property
    def last_update(self):
        return self.last_update_sec + self.last_update_ns / 1e9

    # It is not recommended to use this. The Buffer updates automatically on append and appendleft
    @last_update.setter
    def last_update(self, value, ns_value=0):
        self._sec_check(value)
        self.last_update_sec = int(value)
        if ns_value or value == int(value):
            self._ns_check(ns_value)
            self.last_update_ns = int(ns_value)  # int conversion for fools - we allow it, warn only
        else:
            if self._ns_settable:
                self.last_update_ns = int((value-int(value))*1e9) # int conversion for ridiculous length floats
            else:
                self.last_update_ns = 0

    @property
    def triggered(self):
        return self.period < self.last_update

    def _check_update(self, data, dt, delta_ns):
        self.last_update_sec += dt
        self.last_update_ns += delta_ns
        if not self.triggered or (not self._null_settable and not data):
            return False
        if self._continuous:
            while not self.triggered:
                self.last_update_sec -= self._time_limit_sec
                self.last_update_ns -= self._time_limit_ns
        else:
            self.last_update_sec = 0
            self.last_update_ns = 0
        return True

    # This is the main function we need, as this adds the new item to the start of the buffer (earliest element)
    def appendleft(self, x, dt=0, delta_ns=0):
        update = self._check_update(x, dt, delta_ns)
        if update:
            super().appendleft(x)
        return update

    def append(self, x, dt=0, delta_ns=0):
        update = self._check_update(x, dt, delta_ns)
        if update:
            super().append(x)
        return update

    # for only resetting the last update use Object.last_update = 0
    def clear(self):
        super().clear()
        self.last_update = 0

    def copy(self):
        new_object = super().copy()
        new_object._time_limit_sec = self._time_limit_sec
        new_object._time_limit_ns = self._time_limit_ns
        # we also copy the last update time
        new_object.last_update_sec = self.last_update_sec
        new_object.last_update_ns = self.last_update_ns

    # Pure deque functions
    def pop(self):
        return super().pop()

    def popleft(self):
        return super().popleft()


if __name__ == '__main__':
    obstacle_buffer = CircularTimeBuffer(3, 5, null_settable=False)
    print(obstacle_buffer != -1)
    for i in range(20):
        obstacle_buffer.append(3*i, 1,)
        print(obstacle_buffer)
