import threading


class AtomicCounter:
    """An atomic, thread-safe counter.

    From:
    https://gist.github.com/benhoyt/8c8a8d62debe8e5aa5340373f9c509c7#gistcomment-3142969

    >>> counter = AtomicCounter()
    >>> counter.inc()
    1
    >>> counter = AtomicCounter(42.5)
    >>> counter.value
    42.5
    >>> counter.inc()
    43.5
    >>> counter = AtomicCounter()
    >>> def incrementor():
    ...     for i in range(100000):
    ...         counter.inc()
    >>> threads = []
    >>> for i in range(4):
    ...     thread = threading.Thread(target=incrementor)
    ...     thread.start()
    ...     threads.append(thread)
    >>> for thread in threads:
    ...     thread.join()
    >>> counter.value
    400000
    """

    def __init__(self, initial=0):
        """Initialize a new atomic counter to given initial value"""
        self._value = initial
        self._lock = threading.Lock()

    def inc(self):
        """Atomically increment the counter by num and return the new value"""
        with self._lock:
            self._value += 1
            return self._value

    @property
    def value(self):
        return self._value


if __name__ == "__main__":
    import doctest

    doctest.testmod()
