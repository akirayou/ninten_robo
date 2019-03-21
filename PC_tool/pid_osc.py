# -*- coding: utf-8 -*-
"""
Created on Wed Mar 20 06:49:18 2019

@author: akira
"""

from pythonosc import osc_message_builder
from pythonosc import udp_client


if __name__ == "__main__":

  client = udp_client.SimpleUDPClient("ROB.local",9000)

  client.send_message("/pid/ap", float(-500.0))
  client.send_message("/pid/ai", float(-100.0))
  client.send_message("/pid/ad", float(-0.0))
  client.send_message("/pid/am", float(0.03))
  client.send_message("/pid/af", int(1))
  
  client.send_message("/pid/hp", float(-500.0))
  client.send_message("/pid/hi", float(-300.0))
  client.send_message("/pid/hd", float(-0.0))
  client.send_message("/pid/hm", float(0.02))
  client.send_message("/pid/hf", int(1))
