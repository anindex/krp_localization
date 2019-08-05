from __future__ import print_function
import numpy as np
import sys
import os
import subprocess


class WifiDevice:
    """
    Class WifiDevice
    """

    def __init__(self, **kwargs):
        self.probe_completed = True
        self.mode = kwargs.get("mode", "iw")
        self.device_name = kwargs.get("device_name", "wlx74da38d00733")

    def probe_signal(self):
        self.probe_completed = False
        signals = {}

        if self.mode == "nmcli":
            mac_cmd = 'nmcli -t -f BSSID,SIGNAL device wifi list'
            process = subprocess.Popen(mac_cmd.split(),stdout=subprocess.PIPE)
            process.wait()

            for sig in process.stdout:
                sig = sig.replace("\\", "")
                sig = sig.strip("\n").split(":")
                mac = ":".join(sig[:-1])
                strength = int(sig[-1])
                signals[mac] = strength
        elif self.mode == "iw":
            mac_cmd = 'sudo iw ' + self.device_name + ' scan | awk \'/^BSS/{mac = gensub ( /^BSS[[:space:]]*([0-9a-fA-F:]+).*?$/, \"\\1\", \"g\", $0 );}/^[[:space:]]*signal:/{signal = gensub ( /^[[:space:]]*signal:[[:space:]]*(\-?[0-9.]+).*?$/, \"\\1\", \"g\", $0 );printf (\"%s %s\n\", mac, signal);}\''
            process = subprocess.Popen(mac_cmd.split(),stdout=subprocess.PIPE)
            process.wait()

            for sig in process.stdout:
                sig = sig.strip("\n").split(" ")
                mac = sig[0]
                strength = int(sig[1])
                signals[mac] = strength
        else:
            print("Invalid wifi measurement mode! Mode are nmcli or iw")

        self.probe_completed = True
        return signals

if __name__=="__main__":
    device = WifiDevice()

    print('Streaming starting')
    print(device.probe_signal())
