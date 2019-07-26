from __future__ import print_function
import numpy as np
import sys
import os
import subprocess
import argparse


class WifiDevice:
    """
    Class wirelessDevice
    Wireless lan device handler

    Attributes

    iface Name of the wireless device
    """

    def __init__(self, **kwargs):
        self.probesize = kwargs.get('probesize','5')


        self.probe_completed = True

        #os.system("sudo ifconfig " + self.iface + " down")           #turn interface off
        #os.system("sudo iwconfig " + self.iface + " mode monitor")   #change to monitor mode
        #os.system("sudo ifconfig " + self.iface + " up")             #turn interface on

    def probe_signal(self):
        self.probe_completed = False
        signals = {}

        for _ in range(self.probesize):
            mac_cmd = 'nmcli -t -f BSSID,SIGNAL device wifi list'
            process = subprocess.Popen(mac_cmd.split(),stdout=subprocess.PIPE)
            process.wait()

            for sig in process.stdout:
                sig = sig.replace("\\", "")
                sig = sig.strip("\n").split(":")
                mac = ":".join(sig[:-1])
                strength = sig[-1]
                if mac not in signals:
                    signals[mac] = [strength]
                signals[mac].append(strength)

        self.probe_completed = True
        return signals

if __name__=="__main__":
    parser = argparse.ArgumentParser(description='Wifi Signal Strength monitoring')
    parser.add_argument('-s',dest='probesize',default=5,help='probesize')

    args = parser.parse_args()
    device = WifiDevice(**vars(args))

    print('Streaming starting')
    print(device.probe_signal())
