"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.base import BrandSettings
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.sunnypilot.widgets.list_view import toggle_item_sp


class MazdaSettings(BrandSettings):
  def __init__(self):
    super().__init__()

    self.torque_interceptor_toggle = toggle_item_sp(tr("Torque Interceptor (Beta)"), "",
                                                    param="MazdaTorqueInterceptor", callback=self._on_toggle_changed)

    self.items = [self.torque_interceptor_toggle]

  def _on_toggle_changed(self, _):
    self.update_settings()

  def _disabled_msg(self):
    if not ui_state.is_offroad():
      return tr("Enable \"Always Offroad\" in Device panel, or turn vehicle off to toggle.")
    return ""

  def update_settings(self):
    disabled_msg = self._disabled_msg()
    description = tr("Enable this only if a torque interceptor is physically installed. " +
                     "Allows full-speed-range steering on Gen1 Mazda by bypassing the stock EPS low-speed LKAS lockout.")

    self.torque_interceptor_toggle.action_item.set_enabled(ui_state.is_offroad())
    self.torque_interceptor_toggle.set_description(f"<b>{disabled_msg}</b><br><br>{description}" if disabled_msg else description)
