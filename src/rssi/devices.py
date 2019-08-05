from __future__ import print_function
import numpy as np
import sys
import os
import subprocess


class WifiDevice:
    """
    Class WifiDevice
    """

    def __init__(self):
        self.probe_completed = True

    def probe_signal(self):
        self.probe_completed = False
        signals = {}

        mac_cmd = 'nmcli -t -f BSSID,SIGNAL device wifi list'
        process = subprocess.Popen(mac_cmd.split(),stdout=subprocess.PIPE)
        process.wait()

        for sig in process.stdout:
            sig = sig.replace("\\", "")
            sig = sig.strip("\n").split(":")
            mac = ":".join(sig[:-1])
            strength = int(sig[-1])
            signals[mac] = strength

        self.probe_completed = True
        return signals

if __name__=="__main__":
    device = WifiDevice()

    print('Streaming starting')
    print(device.probe_signal())
