class SerialLinkError(RuntimeError):
    pass


class SerialLink:
    FOUND_BYTES = b"1\n"

    def __init__(self, config, backend_factory=None, sleep_fn=None):
        self.config = config
        self._backend_factory = backend_factory or self._default_backend_factory
        self._sleep = sleep_fn or __import__("time").sleep
        self._port = None

    @staticmethod
    def _default_backend_factory(config):
        import serial

        return serial.Serial(
            port=config.device,
            baudrate=config.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0,
            write_timeout=0.1,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )

    def open(self):
        if self._port is not None:
            return
        try:
            self._port = self._backend_factory(self.config)
        except Exception as exc:
            self._port = None
            raise SerialLinkError(f"failed to open UART {self.config.device}") from exc

    def close(self):
        port, self._port = self._port, None
        if port is None:
            return
        try:
            port.close()
        except Exception:
            pass

    def poll_requests(self):
        if self._port is None:
            raise SerialLinkError("serial port is not open")
        try:
            data = self._port.read(64)
        except Exception as exc:
            raise SerialLinkError("UART read failed") from exc
        requests = []
        for value in bytes(data or b""):
            if value == ord("1"):
                requests.append("red")
            elif value == ord("2"):
                requests.append("blue")
        return requests

    def send_found(self):
        if self._port is None:
            raise SerialLinkError("serial port is not open")
        try:
            written = self._port.write(self.FOUND_BYTES)
        except Exception as exc:
            raise SerialLinkError("UART write failed") from exc
        if written != len(self.FOUND_BYTES):
            raise SerialLinkError(f"short UART write: {written}")

    def reconnect(self):
        self.close()
        for attempt in range(self.config.reconnect_attempts):
            try:
                self.open()
                return True
            except SerialLinkError:
                if attempt + 1 < self.config.reconnect_attempts:
                    self._sleep(self.config.reconnect_delay_ms / 1000.0)
        return False
