import unittest

from rdk_vision.config import SerialConfig
from rdk_vision.serial_link import SerialLink, SerialLinkError


class FakePort:
    def __init__(self, reads=None, fail_write=False):
        self.reads = list(reads or [])
        self.writes = []
        self.fail_write = fail_write
        self.closed = False

    def read(self, size):
        return self.reads.pop(0) if self.reads else b""

    def write(self, data):
        if self.fail_write:
            raise OSError("write failed")
        self.writes.append(bytes(data))
        return len(data)

    def close(self):
        self.closed = True


class SerialLinkTests(unittest.TestCase):
    def config(self):
        return SerialConfig("/dev/fake", 115200, 3, 0)

    def test_parses_raw_request_bytes_without_newline(self):
        port = FakePort([b"1x2\r\n1"])
        link = SerialLink(self.config(), backend_factory=lambda _: port)
        link.open()
        self.assertEqual(link.poll_requests(), ["red", "blue", "red"])

    def test_send_found_writes_exactly_one_ascii_line(self):
        port = FakePort()
        link = SerialLink(self.config(), backend_factory=lambda _: port)
        link.open()
        link.send_found()
        self.assertEqual(port.writes, [b"1\n"])

    def test_write_failure_becomes_serial_link_error(self):
        port = FakePort(fail_write=True)
        link = SerialLink(self.config(), backend_factory=lambda _: port)
        link.open()
        with self.assertRaises(SerialLinkError):
            link.send_found()

    def test_reconnect_uses_finite_attempts_and_succeeds_on_third(self):
        attempts = []
        good_port = FakePort()

        def factory(_config):
            attempts.append(len(attempts) + 1)
            if len(attempts) < 3:
                raise OSError("port unavailable")
            return good_port

        link = SerialLink(
            self.config(),
            backend_factory=factory,
            sleep_fn=lambda _seconds: None,
        )
        self.assertTrue(link.reconnect())
        self.assertEqual(attempts, [1, 2, 3])


if __name__ == "__main__":
    unittest.main()
