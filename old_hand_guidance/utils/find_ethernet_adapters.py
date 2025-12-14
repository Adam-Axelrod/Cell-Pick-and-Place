"""Utility to find network adapters without requiring pysoem.

Provides a `find_adapters()` function that returns a list of objects with
attributes `name` and `desc`. It will use pysoem.find_adapters() when
pysoem is installed and fall back to the standard library (socket.if_nameindex)
otherwise. This keeps the GUI usable on systems without pysoem.
"""
from collections import namedtuple
import socket

Adapter = namedtuple("Adapter", ["name", "desc"])


def find_adapters():
    """Return a list of adapters with `.name` and `.desc` attributes.

    Prefer pysoem if available (it provides EtherCAT-specific adapter info).
    If pysoem isn't present, return OS interface names via socket.if_nameindex().
    """
    try:
        import pysoem

        adapters = pysoem.find_adapters()
        # pysoem adapters often expose `name` and `desc` attributes already.
        return adapters
    except Exception:
        # Fallback: use socket.if_nameindex() available in the standard library
        try:
            names = [n for _, n in socket.if_nameindex()]
        except Exception:
            names = []

        return [Adapter(name=n, desc=n) for n in names]


if __name__ == "__main__":
    for i, a in enumerate(find_adapters()):
        print(f"Adapter {i}")
        print(f"  name: {a.name}")
        print(f"  desc: {getattr(a, 'desc', '')}")
