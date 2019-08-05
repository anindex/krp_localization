from __future__ import print_function
import numpy as np
import sys
import os
import subprocess
import re


class WifiDevice:
    """
    Class WifiDevice
    """

    def __init__(self, **kwargs):
        self.probe_completed = True
        self.mode = kwargs.get("mode", "iw")
        self.device_name = kwargs.get("device_name", "wlx74da38d00733")

        self.cellNumberRe = re.compile(r"^Cell\s+(?P<cellnumber>.+)\s+-\s+Address:\s(?P<mac>.+)$")
        self.signalRe     = re.compile(r"^Quality=(?P<signal_quality>\d+)/(?P<signal_total>\d+)\s+Signal level=(?P<signal_level_dBm>.+) d.+$")

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
            mac_cmd = "iw " + self.device_name + " scan | awk '/^BSS/{mac = gensub ( /^BSS[[:space:]]*([0-9a-fA-F:]+).*?$/, \"\\\\1\", \"g\", $0 );}/^[[:space:]]*signal:/{signal = gensub ( /^[[:space:]]*signal:[[:space:]]*(\-?[0-9.]+).*?$/, \"\\\\1\", \"g\", $0 );printf (\"%s %s \", mac, signal);}'"
            process = subprocess.Popen(mac_cmd,shell=True,stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            sig = process.communicate()[0].split()

            for i in range(0, len(sig) - 1, 2):
                mac, strength = sig[i], int(sig[i + 1])
                signals[mac] = strength

        elif self.mode == "iwlist":
            process = subprocess.Popen(["iwlist", self.device_name, "scan"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            lines = process.stdout.read().split("\n")
            mac = ""

            for line in lines:
                line = line.strip()
                cellNumber = self.cellNumberRe.search(line)
                if cellNumber is not None:
                    mac = cellNumber.groupdict()["mac"]
                    continue

                signalRe = self.signalRe.search(line)
                if signalRe is not None and mac != "":
                    signals[mac] = int(signalRe.groupdict()["signal_level_dBm"])
                    mac = ""

        else:
            print("Invalid wifi measurement mode! Mode are nmcli or iw")

        self.probe_completed = True
        return signals

if __name__=="__main__":
    device = WifiDevice(device_name="wlp3s0", mode="iwlist")

    print('Streaming starting')
    print(device.probe_signal())
