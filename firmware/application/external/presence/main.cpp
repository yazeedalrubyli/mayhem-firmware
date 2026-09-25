/*
 * Copyright (C) 2026 Yazeed Alrubyli
 *
 * This file is part of PortaPack.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include "ui.hpp"
#include "ui_presence.hpp"
#include "ui_navigation.hpp"
#include "external_app.hpp"

namespace ui::external_app::presence {
void initialize_app(ui::NavigationView& nav) {
    nav.push<PresenceView>();
}
}  // namespace ui::external_app::presence

extern "C" {

__attribute__((section(".external_app.app_presence.application_information"), used)) application_information_t _application_information_presence = {
    /*.memory_location = */ (uint8_t*)0x00000000,
    /*.externalAppEntry = */ ui::external_app::presence::initialize_app,
    /*.header_version = */ CURRENT_HEADER_VERSION,
    /*.app_version = */ VERSION_MD5,

    /*.app_name = */ "Presence",
    /*.bitmap_data = */ {// person between two wave arcs, 16x16, LSB = leftmost pixel
                         0x00, 0x00, 0x80, 0x01, 0xC0, 0x03, 0xC0, 0x03, 0x80, 0x01, 0xC4, 0x23, 0xE2, 0x47, 0xEA, 0x57, 0xEA, 0x57, 0xCA, 0x53, 0x62, 0x46, 0x64, 0x26, 0x60, 0x06, 0x60, 0x06, 0x00, 0x00, 0x00, 0x00},
    /*.icon_color = */ ui::Color::green().v,
    /*.menu_location = */ app_location_t::RX,
    /*.desired_menu_position = */ -1,

    /*.m4_app_tag = portapack::spi_flash::image_tag_capture */ {'P', 'C', 'A', 'P'},
    /*.m4_app_offset = */ 0x00000000,  // will be filled at compile time
};
}
