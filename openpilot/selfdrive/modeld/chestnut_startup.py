import struct
import time
from types import SimpleNamespace

import usb1

from openpilot.common.hardware.usb import CHESTNUT_USB_IDS
from openpilot.selfdrive.modeld.helpers import chestnut_ready


def probe_chestnut_ready() -> bool:
  # modeld is the chestnutState publisher, so that service cannot gate its own
  # startup. Read the same power/link telemetry before opening the GPU instead.
  # EP0 reads need no interface claim, configuration change, or tinygrad device.
  try:
    with usb1.USBContext() as context:
      for vendor, product in CHESTNUT_USB_IDS:
        handle = context.openByVendorIDAndProductID(vendor, product, skip_on_error=True)
        if handle is None:
          continue
        try:
          voltage, _, fault = struct.unpack('<Hh?', bytes(handle.controlRead(0xC0, 0xC0, 0, 0, 5, timeout=100)))
          pcie, = bytes(handle.controlRead(0xC0, 0xE4, 0xB450, 0, 1, timeout=100))
          return chestnut_ready(SimpleNamespace(supplyVoltage=voltage, supplyFault=fault, pcieLtssm=pcie))
        finally:
          handle.close()
  except (usb1.USBError, OSError, ValueError, struct.error):
    return False
  return False


def wait_for_chestnut_ready(timeout: float) -> bool:
  deadline = time.monotonic() + timeout
  while time.monotonic() < deadline:
    if probe_chestnut_ready():
      return True
    time.sleep(min(0.1, max(0., deadline - time.monotonic())))
  return False
