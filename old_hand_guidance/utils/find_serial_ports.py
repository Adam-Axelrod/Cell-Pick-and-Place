"""Prints name and description of available network adapters."""

from serial.tools.list_ports import comports

ports = comports()

for i, port in enumerate(ports):
    print('Port {}'.format(i))
    print('  {}'.format(port.description))
    print('  {}'.format(port.device))
