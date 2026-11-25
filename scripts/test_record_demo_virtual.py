#!/usr/bin/env python3
"""Linux checks for recording process isolation and occupied-port protection."""

import os
import socket
import subprocess
import sys
import time
import unittest
import uuid

from record_demo_virtual import require_free_port, tagged_processes


@unittest.skipUnless(sys.platform == "linux", "Process ownership uses Linux /proc")
class RecordingIsolationTest(unittest.TestCase):
    def test_process_tag_matches_whole_environment_value(self):
        tag = "recording_test_" + uuid.uuid4().hex
        children = []
        try:
            for value in (tag, tag + "_unrelated"):
                children.append(subprocess.Popen(
                    [sys.executable, "-c", "import time; time.sleep(20)"],
                    env=dict(os.environ, UAV_RECORDING_SESSION=value)))
            deadline = time.monotonic() + 2
            while children[0].pid not in tagged_processes(tag) and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertEqual(tagged_processes(tag), [children[0].pid])
            self.assertNotIn(children[1].pid, tagged_processes(tag))
        finally:
            for child in children:
                child.terminate()
                child.wait(timeout=3)
        self.assertEqual(tagged_processes(tag), [])

    def test_active_dashboard_listener_is_rejected(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind(("127.0.0.1", 0))
            listener.listen(1)
            with self.assertRaises(OSError):
                require_free_port(listener.getsockname()[1], socket.SOCK_STREAM)

    def test_active_xrce_udp_socket_is_rejected(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as listener:
            listener.bind(("127.0.0.1", 0))
            with self.assertRaises(OSError):
                require_free_port(listener.getsockname()[1], socket.SOCK_DGRAM)


if __name__ == "__main__":
    unittest.main()
