// license:BSD-3-Clause
// copyright-holders:Danila Sukharev
#ifndef MAME_BUS_PCCARD_3C589_H
#define MAME_BUS_PCCARD_3C589_H

#pragma once

#include "pccard.h"
#include "dinetwork.h"

#include <array>
#include <vector>


class etherlink_iii_pccard_device :
	public device_t,
	public device_pccard_interface,
	public device_network_interface
{
public:
	etherlink_iii_pccard_device(
			machine_config const &mconfig,
			char const *tag,
			device_t *owner,
			u32 clock = 0);

	virtual u16 read_memory(offs_t offset, u16 mem_mask = ~0) override;
	virtual u16 read_reg(offs_t offset, u16 mem_mask = ~0) override;
	virtual void write_memory(offs_t offset, u16 data, u16 mem_mask = ~0) override;
	virtual void write_reg(offs_t offset, u16 data, u16 mem_mask = ~0) override;

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	virtual int recv_start_cb(u8 *buf, int length) override;

private:
	static constexpr u32 CONFIG_BASE = 0x1000;
	static constexpr u16 FIFO_BYTES = 8 * 1024;

	enum status : u16
	{
		INT_LATCH       = 0x0001,
		ADAPTER_FAILURE = 0x0002,
		TX_COMPLETE     = 0x0004,
		TX_AVAILABLE    = 0x0008,
		RX_COMPLETE     = 0x0010,
		RX_EARLY        = 0x0020,
		INT_REQUESTED   = 0x0040,
		STATS_FULL      = 0x0080
	};

	void set_present(bool present);
	void update_irq();
	void execute_command(u16 command);
	u16 io_r(u8 offset, u16 mem_mask);
	void io_w(u8 offset, u16 data, u16 mem_mask);
	u16 register_r(u8 offset);
	void register_w(u8 offset, u16 data, u16 mem_mask);
	u16 fifo_r(u16 mem_mask);
	void fifo_w(u16 data, u16 mem_mask);
	void finish_transmit();
	bool receive_filter_accepts(u8 const *frame, unsigned length) const;
	void discard_receive();

	u8 m_configuration_option = 0;
	u8 m_configuration_status = 0;
	u8 m_window = 0;
	u16 m_configuration_control = 0;
	u16 m_address_configuration = 0;
	u16 m_resource_configuration = 0;
	u16 m_eeprom_command = 0;
	u16 m_eeprom_data = 0;
	u16 m_media_status = 0;
	u16 m_net_diagnostic = 0;
	u16 m_interrupt_enable = 0;
	u16 m_status_enable = 0;
	u16 m_pending = 0;
	u16 m_receive_filter = 0;
	u16 m_receive_threshold = 0;
	u16 m_transmit_threshold = 0;
	u16 m_transmit_start = 0;
	std::array<u8, 6> m_station_address{};
	std::array<u16, 64> m_eeprom{};
	std::vector<u8> m_transmit_data;
	std::vector<u8> m_receive_data;
	u16 m_transmit_length = 0;
	u16 m_transmit_header_words = 0;
	u16 m_receive_position = 0;
	bool m_receiver_enabled = false;
	bool m_transmitter_enabled = false;
	bool m_statistics_enabled = false;
};

DECLARE_DEVICE_TYPE(ETHERLINK_III_PCCARD, etherlink_iii_pccard_device)

#endif // MAME_BUS_PCCARD_3C589_H
