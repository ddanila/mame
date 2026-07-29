// license:BSD-3-Clause
// copyright-holders:Danila Sukharev
/***************************************************************************

    General Magic DataRover 840

    Hardware:
      - Toshiba TMPR3902U (TX39/R3900), 36.864 MHz
      - 4 MiB DRAM
      - 8 MiB ROM window
      - 480x320 grayscale LCD
      - TX39 "Dino" integrated peripheral block
      - two "Glacier" GPIO/interrupt blocks
      - General Magic "Betty" peripheral ASIC connected over SIB

    The published MagicCap-USA.image is raw ROM content beginning at
    physical 0x13c00000.  The CPU reset vector at 0xbfc00000 aliases ROM
    offset zero; its first pseudo-direct jump continues at the regular
    uncached ROM alias, 0xb3c0001c.

    The R3900 CPU device supplies the documented PRId, cache sizes and
    TLB-less kuseg mapping.  Dino, Glacier and Betty are early behavioural
    stubs derived from the matching unstripped Icras SDK ELF.

    TODO:
      - add the remaining TX39-specific CP0 and MAC behaviour
      - replace Dino register shadows with functional devices

***************************************************************************/

#include "emu.h"

#include "bus/pccard/3c589.h"
#include "bus/pccard/pccard.h"
#include "bus/rs232/rs232.h"
#include "cpu/mips/mips1.h"
#include "diserial.h"
#include "dipty.h"
#include "machine/intelfsh.h"
#include "machine/nvram.h"
#include "machine/pckeybrd.h"
#include "machine/terminal.h"
#include "sound/dmadac.h"

#include "screen.h"
#include "speaker.h"

#include "datarover840.lh"

#include <array>
#include <vector>


class datarover_uart_device;
DECLARE_DEVICE_TYPE(DATAROVER_UART, datarover_uart_device)
class datarover_irda_device;
DECLARE_DEVICE_TYPE(DATAROVER_IRDA, datarover_irda_device)

class datarover_uart_device :
	public device_t,
	public device_buffered_serial_interface<65'536U>
{
public:
	datarover_uart_device(
			machine_config const &mconfig,
			char const *tag,
			device_t *owner,
			u32 clock = 0);

	auto txd_handler() { return m_txd_handler.bind(); }
	auto received_handler() { return m_received_handler.bind(); }

	void input_txd(int state)
	{
		device_buffered_serial_interface::rx_w(state);
	}

	void transmit(u8 data)
	{
		transmit_byte(data);
	}

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void tra_callback() override;
	virtual void tra_complete() override;
	virtual void received_byte(u8 byte) override;

private:
	devcb_write_line m_txd_handler;
	devcb_write8 m_received_handler;
};

DEFINE_DEVICE_TYPE(
		DATAROVER_UART,
		datarover_uart_device,
		"datarover_uart",
		"DataRover Dino UART")

datarover_uart_device::datarover_uart_device(
		machine_config const &mconfig,
		char const *tag,
		device_t *owner,
		u32 clock)
	: device_t(mconfig, DATAROVER_UART, tag, owner, clock)
	, device_buffered_serial_interface(mconfig, *this)
	, m_txd_handler(*this)
	, m_received_handler(*this)
{
}

void datarover_uart_device::device_start()
{
}

void datarover_uart_device::device_reset()
{
	clear_fifo();
	set_data_frame(1, 8, PARITY_NONE, STOP_BITS_1);
	set_tra_rate(19'200);
	set_rcv_rate(19'200);
	receive_register_reset();
	transmit_register_reset();
	m_txd_handler(1);
}

void datarover_uart_device::tra_callback()
{
	m_txd_handler(transmit_register_get_data_bit());
}

void datarover_uart_device::tra_complete()
{
	device_buffered_serial_interface::tra_complete();
}

void datarover_uart_device::received_byte(u8 byte)
{
	m_received_handler(byte);
}


class datarover_irda_device :
	public device_t,
	public device_pty_interface
{
public:
	datarover_irda_device(
			machine_config const &mconfig,
			char const *tag,
			device_t *owner,
			u32 clock = 0);

	auto received_handler() { return m_received_handler.bind(); }

	void transmit(u8 data)
	{
		write(data);
	}

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_stop() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

private:
	TIMER_CALLBACK_MEMBER(poll);

	devcb_write8 m_received_handler;
	emu_timer *m_poll_timer = nullptr;
};

DEFINE_DEVICE_TYPE(
		DATAROVER_IRDA,
		datarover_irda_device,
		"datarover_irda",
		"DataRover IrDA SIR Port")

datarover_irda_device::datarover_irda_device(
		machine_config const &mconfig,
		char const *tag,
		device_t *owner,
		u32 clock)
	: device_t(mconfig, DATAROVER_IRDA, tag, owner, clock)
	, device_pty_interface(mconfig, *this)
	, m_received_handler(*this)
{
}

void datarover_irda_device::device_start()
{
	open();
	m_poll_timer = timer_alloc(FUNC(datarover_irda_device::poll), this);
	m_poll_timer->adjust(attotime::zero, 0, attotime::from_msec(1));
}

void datarover_irda_device::device_stop()
{
	close();
}

void datarover_irda_device::device_reset()
{
}

TIMER_CALLBACK_MEMBER(datarover_irda_device::poll)
{
	u8 input[1024];
	ssize_t const count = read(input, std::size(input));
	if (count > 0)
	{
		for (ssize_t index = 0; index < count; ++index)
			m_received_handler(input[index]);
	}
}


class datarover_modem_pccard_device;
DECLARE_DEVICE_TYPE(DATAROVER_MODEM_PCCARD, datarover_modem_pccard_device)

class datarover_modem_pccard_device :
	public device_t,
	public device_pccard_interface,
	public device_pty_interface
{
public:
	datarover_modem_pccard_device(
			machine_config const &mconfig,
			char const *tag,
			device_t *owner,
			u32 clock = 0);

	virtual u16 read_memory(offs_t offset, u16 mem_mask = ~0) override;
	virtual u16 read_reg(offs_t offset, u16 mem_mask = ~0) override;
	virtual void write_memory(offs_t offset, u16 data, u16 mem_mask = ~0) override;
	virtual void write_reg(offs_t offset, u16 data, u16 mem_mask = ~0) override;
	void poll() { poll_pty(); }
	void restore_state(bool reinsert);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_stop() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

private:
	void set_present(bool present);
	void poll_pty();
	void update_irq();
	u8 memory_byte_r(u32 address);
	void memory_byte_w(u32 address, u8 data);

	static constexpr u32 UART_BASE = 0x03f8;
	static constexpr u32 CONFIG_BASE = 0x0100;
	static constexpr u32 RX_QUEUE_SIZE = 65'536;

	std::array<u8, RX_QUEUE_SIZE> m_rx_data{};
	u32 m_rx_head = 0;
	u32 m_rx_count = 0;
	u8 m_config_option = 0;
	u8 m_ier = 0;
	u16 m_divisor = 0;
	u8 m_fcr = 0;
	u8 m_lcr = 0;
	u8 m_mcr = 0;
	u8 m_scratch = 0;
	bool m_tx_irq_pending = false;
};

DEFINE_DEVICE_TYPE(
		DATAROVER_MODEM_PCCARD,
		datarover_modem_pccard_device,
		"datarover_modem_pccard",
		"DataRover PC Card Hayes Modem")

datarover_modem_pccard_device::datarover_modem_pccard_device(
		machine_config const &mconfig,
		char const *tag,
		device_t *owner,
		u32 clock)
	: device_t(mconfig, DATAROVER_MODEM_PCCARD, tag, owner, clock)
	, device_pccard_interface(mconfig, *this)
	, device_pty_interface(mconfig, *this)
{
}

void datarover_modem_pccard_device::device_start()
{
	open();

	save_item(NAME(m_rx_data));
	save_item(NAME(m_rx_head));
	save_item(NAME(m_rx_count));
	save_item(NAME(m_config_option));
	save_item(NAME(m_ier));
	save_item(NAME(m_divisor));
	save_item(NAME(m_fcr));
	save_item(NAME(m_lcr));
	save_item(NAME(m_mcr));
	save_item(NAME(m_scratch));
	save_item(NAME(m_tx_irq_pending));
}

void datarover_modem_pccard_device::set_present(bool present)
{
	m_cd1_cb(present ? 0 : 1);
	m_cd2_cb(present ? 0 : 1);
	m_bvd1_cb(1);
	m_bvd2_cb(1);
	m_wp_cb(0);
}

void datarover_modem_pccard_device::restore_state(bool reinsert)
{
	if (reinsert)
	{
		// A state created without this optional device describes an empty slot.
		// Selecting the modem for that launch is a real insertion.
		set_present(false);
		set_present(true);
	}
	else
	{
		// A state saved with this modem present must retain continuous card
		// presence. Re-drive levels without a false remove/insert or IREQ pulse.
		m_cd1_cb(0);
		m_cd2_cb(0);
		m_bvd2_cb(1);
		m_wp_cb(0);
	}
	update_irq();
}

void datarover_modem_pccard_device::device_stop()
{
	close();
}

void datarover_modem_pccard_device::device_reset()
{
	m_rx_data.fill(0);
	m_rx_head = 0;
	m_rx_count = 0;
	m_config_option = 0;
	m_ier = 0;
	m_divisor = 0;
	m_fcr = 0;
	m_lcr = 0;
	m_mcr = 0;
	m_scratch = 0;
	m_tx_irq_pending = false;
	set_present(true);
	update_irq();
}

void datarover_modem_pccard_device::update_irq()
{
	// In PC Card I/O mode BVD1 becomes the active-low IREQ signal.
	bool const pending =
			((m_ier & 0x01) && m_rx_count)
			|| ((m_ier & 0x02) && m_tx_irq_pending);
	m_bvd1_cb(pending ? 0 : 1);
}

void datarover_modem_pccard_device::poll_pty()
{
	u8 input[1024];
	ssize_t const count = read(input, std::size(input));
	if (count > 0)
	{
		for (ssize_t index = 0; index < count; ++index)
		{
			if (m_rx_count == RX_QUEUE_SIZE)
				break;
			m_rx_data[(m_rx_head + m_rx_count) % RX_QUEUE_SIZE] = input[index];
			++m_rx_count;
		}
		update_irq();
	}
}

u8 datarover_modem_pccard_device::memory_byte_r(u32 address)
{
	if ((address < UART_BASE) || (address >= (UART_BASE + 8)))
		return 0xff;

	poll_pty();
	unsigned const reg = address - UART_BASE;
	switch (reg)
	{
	case 0:
		if (BIT(m_lcr, 7))
			return u8(m_divisor);
		if (m_rx_count)
		{
			u8 const result = m_rx_data[m_rx_head];
			m_rx_head = (m_rx_head + 1) % RX_QUEUE_SIZE;
			--m_rx_count;
			update_irq();
			return result;
		}
		return 0;
	case 1:
		return BIT(m_lcr, 7) ? u8(m_divisor >> 8) : m_ier;
	case 2:
		if ((m_ier & 0x01) && m_rx_count)
			return (m_fcr & 0x01) ? 0xc4 : 0x04;
		if ((m_ier & 0x02) && m_tx_irq_pending)
		{
			m_tx_irq_pending = false;
			update_irq();
			return (m_fcr & 0x01) ? 0xc2 : 0x02;
		}
		return (m_fcr & 0x01) ? 0xc1 : 0x01;
	case 3:
		return m_lcr;
	case 4:
		return m_mcr;
	case 5:
		return 0x60 | (m_rx_count ? 0x01 : 0x00);
	case 6:
		return 0xb0; // DCD, DSR and CTS asserted
	case 7:
		return m_scratch;
	}
	return 0xff;
}

void datarover_modem_pccard_device::memory_byte_w(u32 address, u8 data)
{
	if ((address < UART_BASE) || (address >= (UART_BASE + 8)))
		return;

	unsigned const reg = address - UART_BASE;
	switch (reg)
	{
	case 0:
		if (BIT(m_lcr, 7))
			m_divisor = (m_divisor & 0xff00) | data;
		else
		{
			write(data);
			m_tx_irq_pending = true;
		}
		break;
	case 1:
		if (BIT(m_lcr, 7))
			m_divisor = (m_divisor & 0x00ff) | (u16(data) << 8);
		else
		{
			m_ier = data & 0x0f;
			if (m_ier & 0x02)
				m_tx_irq_pending = true;
		}
		break;
	case 2:
		m_fcr = data;
		if (data & 0x02)
		{
			m_rx_head = 0;
			m_rx_count = 0;
		}
		break;
	case 3:
		m_lcr = data;
		break;
	case 4:
		m_mcr = data;
		break;
	case 7:
		m_scratch = data;
		break;
	default:
		break;
	}
	update_irq();
}

u16 datarover_modem_pccard_device::read_memory(offs_t offset, u16 mem_mask)
{
	u32 const address = offset * 2;
	u16 result = 0xffff;
	if (mem_mask & 0x00ff)
		result = (result & 0xff00) | memory_byte_r(address);
	if (mem_mask & 0xff00)
		result = (result & 0x00ff) | (u16(memory_byte_r(address + 1)) << 8);
	return result;
}

void datarover_modem_pccard_device::write_memory(
		offs_t offset,
		u16 data,
		u16 mem_mask)
{
	u32 const address = offset * 2;
	if (mem_mask & 0x00ff)
		memory_byte_w(address, u8(data));
	if (mem_mask & 0xff00)
		memory_byte_w(address + 1, u8(data >> 8));
}

u16 datarover_modem_pccard_device::read_reg(offs_t offset, u16 mem_mask)
{
	// Glacier multiplexes attribute and I/O cycles into this card window.
	// Attribute bytes are spaced two host addresses apart, so the PC Card
	// interface's word offset maps back to the conventional byte I/O address.
	u32 const io_address = offset * 2;
	if ((io_address >= UART_BASE) && (io_address < (UART_BASE + 8)))
		return read_memory(offset, mem_mask);

	// Minimal standards-compliant CIS for an I/O card with a serial function.
	// Magic Cap identifies a modem by CISTPL_FUNCID value 2, reads the
	// configuration-register base from CISTPL_CONFIG, and maps the 16550 at
	// the conventional COM1 I/O address.
	static constexpr std::array<u8, 61> CIS{
			0x01, 0x04, 0xdf, 0x4a, 0x01, 0xff,
			0x15, 0x1e, 0x05, 0x00,
			'G', 'e', 'n', 'e', 'r', 'a', 'l', ' ', 'M', 'a', 'g', 'i', 'c', 0x00,
			'P', 'C', ' ', 'C', 'a', 'r', 'd', ' ', 'M', 'o', 'd', 'e', 'm', 0x00,
			0x21, 0x02, 0x02, 0x00,
			0x1a, 0x05, 0x01, 0x01, 0x00, 0x02, 0x01,
			// Default I/O configuration: 10 decoded address lines, one
			// eight-byte 8/16-bit range at the conventional COM1 base.
			0x1b, 0x09, 0xc1, 0x01, 0x08, 0xaa, 0x60,
			0xf8, 0x03, 0x07, 0xff,
			0xff };

	u8 value = 0xff;
	if (offset < CIS.size())
		value = CIS[offset];
	else if (offset == CONFIG_BASE)
		value = m_config_option;
	return (mem_mask & 0x00ff) ? (0xff00 | value) : 0xffff;
}

void datarover_modem_pccard_device::write_reg(
		offs_t offset,
		u16 data,
		u16 mem_mask)
{
	u32 const io_address = offset * 2;
	if ((io_address >= UART_BASE) && (io_address < (UART_BASE + 8)))
	{
		write_memory(offset, data, mem_mask);
		return;
	}

	if ((offset == CONFIG_BASE) && (mem_mask & 0x00ff))
		m_config_option = u8(data);
}


class datarover_linear_pccard_device;
DECLARE_DEVICE_TYPE(DATAROVER_LINEAR_PCCARD, datarover_linear_pccard_device)

class datarover_linear_pccard_device :
	public device_t,
	public device_image_interface,
	public device_pccard_interface
{
public:
	datarover_linear_pccard_device(
			machine_config const &mconfig,
			char const *tag,
			device_t *owner,
			u32 clock = 0);

	virtual u16 read_memory(offs_t offset, u16 mem_mask = ~0) override;
	virtual u16 read_reg(offs_t offset, u16 mem_mask = ~0) override;
	virtual void write_memory(offs_t offset, u16 data, u16 mem_mask = ~0) override;

protected:
	virtual void device_start() override ATTR_COLD;

	virtual bool is_readable() const noexcept override { return true; }
	virtual bool is_writeable() const noexcept override { return true; }
	virtual bool is_creatable() const noexcept override { return true; }
	virtual bool is_reset_on_load() const noexcept override { return false; }
	virtual bool support_command_line_image_creation() const noexcept override { return true; }
	virtual char const *file_extensions() const noexcept override { return "bin,card,img"; }
	virtual char const *image_type_name() const noexcept override { return "linearcard"; }
	virtual char const *image_brief_type_name() const noexcept override { return "card"; }
	virtual std::pair<std::error_condition, std::string> call_load() override;
	virtual std::pair<std::error_condition, std::string> call_create(
			int format_type,
			util::option_resolution *format_options) override;
	virtual void call_unload() override;

private:
	static constexpr u32 CARD_SIZE = 8 * 1024 * 1024;
	static constexpr u32 LEGACY_WRAPPER_SIZE = 0x70;
	static constexpr u32 LEGACY_CIS_OFFSET = 0x0c;
	void set_present(bool present);
	void flush();
	bool has_storage_header() const;
	bool has_legacy_storage_header() const;

	std::vector<u8> m_data;
	bool m_dirty = false;
	bool m_magic_cap_storage = false;
	bool m_legacy_storage = false;
};

DEFINE_DEVICE_TYPE(
		DATAROVER_LINEAR_PCCARD,
		datarover_linear_pccard_device,
		"datarover_linear_pccard",
		"DataRover 8 MiB Linear Memory PC Card")

datarover_linear_pccard_device::datarover_linear_pccard_device(
		machine_config const &mconfig,
		char const *tag,
		device_t *owner,
		u32 clock)
	: device_t(mconfig, DATAROVER_LINEAR_PCCARD, tag, owner, clock)
	, device_image_interface(mconfig, *this)
	, device_pccard_interface(mconfig, *this)
	, m_data(CARD_SIZE, 0xff)
{
}

void datarover_linear_pccard_device::device_start()
{
	save_item(NAME(m_data));
	save_item(NAME(m_dirty));
	save_item(NAME(m_magic_cap_storage));
	save_item(NAME(m_legacy_storage));
	set_present(exists());
}

void datarover_linear_pccard_device::set_present(bool present)
{
	m_cd1_cb(present ? 0 : 1);
	m_cd2_cb(present ? 0 : 1);
	m_bvd1_cb(1);
	m_bvd2_cb(1);
	m_wp_cb(present && is_readonly());
}

std::pair<std::error_condition, std::string> datarover_linear_pccard_device::call_load()
{
	if (length() != CARD_SIZE)
		return std::make_pair(
				image_error::INVALIDLENGTH,
				"DataRover linear card images must be exactly 8 MiB");

	if (fread(m_data.data(), m_data.size()) != m_data.size())
		return std::make_pair(image_error::UNSPECIFIED, "Unable to read card image");

	m_legacy_storage = has_legacy_storage_header();
	m_magic_cap_storage =
			std::all_of(m_data.begin(), m_data.end(), [](u8 value) { return value == 0xff; })
			|| has_storage_header()
			|| m_legacy_storage;
	m_dirty = false;
	set_present(true);
	return std::make_pair(std::error_condition(), std::string());
}

std::pair<std::error_condition, std::string> datarover_linear_pccard_device::call_create(
		int format_type,
		util::option_resolution *format_options)
{
	(void)format_type;
	(void)format_options;

	std::fill(m_data.begin(), m_data.end(), 0xff);
	m_magic_cap_storage = true;
	m_legacy_storage = false;
	if (fwrite(m_data.data(), m_data.size()) != m_data.size())
		return std::make_pair(image_error::UNSPECIFIED, "Unable to create card image");

	m_dirty = false;
	set_present(true);
	return std::make_pair(std::error_condition(), std::string());
}

void datarover_linear_pccard_device::flush()
{
	if (m_dirty && !is_readonly())
	{
		fseek(0, SEEK_SET);
		fwrite(m_data.data(), m_data.size());
		m_dirty = false;
	}
}

void datarover_linear_pccard_device::call_unload()
{
	flush();
	std::fill(m_data.begin(), m_data.end(), 0xff);
	m_magic_cap_storage = false;
	m_legacy_storage = false;
	set_present(false);
}

bool datarover_linear_pccard_device::has_storage_header() const
{
	return m_data.size() >= 0x5c
			&& std::equal(m_data.begin() + 0x58, m_data.begin() + 0x5c, "MCAP");
}

bool datarover_linear_pccard_device::has_legacy_storage_header() const
{
	// Magic Cap Simulator 1.x GMCD/MCAP files keep their Macintosh-facing
	// CIS at the beginning of the image.  tools/legacy_card_image.py merges
	// the optional changes file and pads that representation to 8 MiB.
	static constexpr std::array<u8, 6> LEGACY_GM_TUPLE{
			0xa0, 0x20, 'G', 'M', 'M', 'C' };
	return m_data.size() >= 0x23
			&& std::equal(
					LEGACY_GM_TUPLE.begin(),
					LEGACY_GM_TUPLE.end(),
					m_data.begin() + 0x1d);
}

u16 datarover_linear_pccard_device::read_memory(offs_t offset, u16 mem_mask)
{
	// A Simulator card's CISTPL_GM tuple advertises the metacluster's offset
	// in the combined file (normally 0x70).  Keep that same layout in common
	// memory: stripping the wrapper here would apply the offset twice and
	// make the 1.0 translator scan 0x70 bytes past the metacluster.
	u32 const address = offset * 2;
	u16 result = 0xffff;
	if (address < m_data.size())
	{
		if (mem_mask & 0x00ff)
			result = (result & 0xff00) | m_data[address];
		if ((mem_mask & 0xff00) && ((address + 1) < m_data.size()))
			result = (result & 0x00ff) | (u16(m_data[address + 1]) << 8);
	}
	return result;
}

void datarover_linear_pccard_device::write_memory(offs_t offset, u16 data, u16 mem_mask)
{
	u32 const address = offset * 2;
	if (is_readonly() || (address >= m_data.size()))
		return;

	if (mem_mask & 0x00ff)
		m_data[address] = u8(data);
	if ((mem_mask & 0xff00) && ((address + 1) < m_data.size()))
		m_data[address + 1] = u8(data >> 8);
	m_dirty = true;
}

u16 datarover_linear_pccard_device::read_reg(offs_t offset, u16 mem_mask)
{
	// Raw images that are neither erased nor Magic Cap volumes retain the
	// generic SRAM identity used by the low-level PC Card probe and ROM
	// flasher.  Magic Cap deliberately refuses to overwrite such a valid,
	// unknown card.
	static constexpr std::array<u8, 42> GENERIC_CIS{
			0x01, 0x03, 0x61, 0x7d, 0xff,
			0x15, 0x1e, 0x04, 0x01,
			'G', 'e', 'n', 'e', 'r', 'a', 'l', ' ', 'M', 'a', 'g', 'i', 'c', 0x00,
			'L', 'i', 'n', 'e', 'a', 'r', ' ', 'M', 'e', 'm', 'o', 'r', 'y', 0x00,
			0x21, 0x02, 0x01, 0x00,
			0xff };

	// General Magic CISTPL_GM is supplied separately from common memory,
	// just as it is on a physical Magic Cap storage card.  A blank card
	// must not also advertise a conventional valid SRAM CIS, or the OS
	// correctly treats it as an unknown card that must not be erased.
	std::array<u8, 35> magic_cap_cis{
			// Blank RAM card, with no common-memory metacluster yet.  The
			// eight payload words are big-endian as documented by the Magic
			// Cap PC Cards developer FAQ.
			0xa0, 0x20,
			'G', 'M', 'M', 'C',
			0x00, 0x01, 0x00, 0x01,
			'B', 'L', 'N', 'K',
			0x00, 0x00, 0x00, 0x00,
			'D', 'R', '8', '4',
			0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00,
			0xff };

	if (has_storage_header())
	{
		std::copy_n("RAMC", 4, magic_cap_cis.begin() + 10);
		std::copy_n(m_data.begin() + 0x60, 4, magic_cap_cis.begin() + 14);
	}

	// A Simulator 1.x card keeps a 12-byte Macintosh wrapper before the
	// authentic tuple stream.  Expose the CIS that follows it as attribute
	// memory while retaining the combined offsets in common memory.
	u8 const value = m_legacy_storage
			? (((offset + LEGACY_CIS_OFFSET) < LEGACY_WRAPPER_SIZE)
					? m_data[offset + LEGACY_CIS_OFFSET]
					: 0xff)
			: m_magic_cap_storage
			? ((offset < magic_cap_cis.size()) ? magic_cap_cis[offset] : 0xff)
			: ((offset < GENERIC_CIS.size()) ? GENERIC_CIS[offset] : 0xff);
	return (mem_mask & 0x00ff) ? (0xff00 | value) : 0xffff;
}


namespace {

class datarover_state : public driver_device, public device_sound_interface
{
public:
	datarover_state(machine_config const &mconfig, device_type type, char const *tag)
		: driver_device(mconfig, type, tag)
		, device_sound_interface(mconfig, *this)
		, m_maincpu(*this, "maincpu")
		, m_screen(*this, "screen")
		, m_terminal(*this, "terminal")
		, m_uart(*this, "uart%u", 1U)
		, m_rs232(*this, "rs232%u", 1U)
		, m_irda(*this, "irda")
		, m_dmadac(*this, "speaker_dac")
		, m_microphone(*this, "microphone")
		, m_magicbus_keyboard(*this, "magicbus_keyboard")
		, m_pccard(*this, "pccard%u", 1U)
		, m_modem_card(*this, "pccard%u:modem", 1U)
		, m_flash(*this, "flash%u", 0U)
		, m_rtc_nvram(*this, "rtc")
		, m_ram(*this, "ram")
		, m_rom(*this, "maincpu")
		, m_boot_mode(*this, "BOOT_MODE")
		, m_rtc_resume(*this, "RTC_RESUME")
		, m_battery(*this, "BATTERY")
		, m_power_supply(*this, "POWER_SUPPLY")
		, m_option_button(*this, "OPTION_BUTTON")
		, m_pccard_battery(*this, "PCCARD%u_BATTERY", 1U)
		, m_touch_x(*this, "TOUCH_X")
		, m_touch_y(*this, "TOUCH_Y")
		, m_touch_button(*this, "TOUCH_BUTTON")
		, m_power_button(*this, "POWER_BUTTON")
		, m_phone_line(*this, "PHONE_LINE")
		, m_phone_ring(*this, "PHONE_RING")
		, m_magicbus_accessory(*this, "MAGICBUS_ACCESSORY")
		, m_irda_carrier(*this, "IRDA_CARRIER")
		, m_microphone_source(*this, "MICROPHONE_SOURCE")
	{
	}

	void datarover840(machine_config &config);
	void datarover840f(machine_config &config);
	INPUT_CHANGED_MEMBER(touch_changed);
	INPUT_CHANGED_MEMBER(option_changed);
	INPUT_CHANGED_MEMBER(pccard_battery_changed);
	INPUT_CHANGED_MEMBER(power_changed);
	INPUT_CHANGED_MEMBER(phone_line_changed);
	INPUT_CHANGED_MEMBER(phone_ring_changed);
	INPUT_CHANGED_MEMBER(main_battery_changed);
	INPUT_CHANGED_MEMBER(battery_cover_changed);
	INPUT_CHANGED_MEMBER(irda_carrier_changed);

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;
	virtual void sound_stream_update(sound_stream &stream) override;

private:
	static constexpr u32 DINO_UART_A_CONTROL1 = 0x0b0 / 4;
	static constexpr u32 DINO_UART_A_HOLD = 0x0c4 / 4;
	static constexpr u32 DINO_UART_B_CONTROL1 = 0x0c8 / 4;
	static constexpr u32 DINO_UART_B_HOLD = 0x0dc / 4;
	static constexpr u32 DINO_UART_ENABLED_STATUS = 0x8000'0000;
	static constexpr u32 DINO_UART_EMPTY_STATUS = 0x4000'0000;
	static constexpr u32 DINO_UART_PRX_HOLD_FULL = 0x2000'0000;
	static constexpr u32 DINO_UART_RX_HOLD_FULL = 0x1000'0000;
	static constexpr u32 DINO_UART_STATUS =
			DINO_UART_ENABLED_STATUS | DINO_UART_EMPTY_STATUS
			| DINO_UART_PRX_HOLD_FULL | DINO_UART_RX_HOLD_FULL;
	static constexpr u32 DINO_UART_PULSED_MODE = 0x0000'0300;
	static constexpr u32 DINO_SIB_SF0_AUX = 0x080 / 4;
	static constexpr u32 DINO_SIB_SF1_AUX = 0x084 / 4;
	static constexpr u32 DINO_SIB_SF0_STATUS = 0x088 / 4;
	static constexpr u32 DINO_SIB_SF1_STATUS = 0x08c / 4;
	static constexpr u32 DINO_SIB_SOUND_HOLD = 0x078 / 4;
	static constexpr u32 DINO_SIB_CONTROL = 0x074 / 4;
	static constexpr u32 DINO_SIB_SIZE = 0x060 / 4;
	static constexpr u32 DINO_SIB_SOUND_RX_START = 0x064 / 4;
	static constexpr u32 DINO_SIB_SOUND_TX_START = 0x068 / 4;
	static constexpr u32 DINO_SIB_DMA = 0x090 / 4;
	static constexpr u32 SIB_STATUS_AUDIO_VALID = 0x0002'0000;

	// DinoModule.sibDMA, named by the SDK's Dino.asm.h.
	static constexpr u32 SIB_SOUND_DMA_ONCE = 0x8000'0000; // kSibSoundDmaOnceMask
	static constexpr u32 SIB_SOUND_DMA_LOOP = 0x4000'0000; // kSibSoundDmaLoopMask
	static constexpr u32 SIB_SOUND_DMA_PTR = 0x3ffc'0000;  // kSibSoundDmaPtrMask
	static constexpr u32 SIB_SOUND_RX_DMA_EN = 0x0002'0000; // kSibEnSoundRxDmaMask
	static constexpr u32 SIB_SOUND_TX_DMA_EN = 0x0001'0000; // kSibEnSoundTxDmaMask
	static constexpr unsigned SIB_SOUND_PTR_SHIFT = 18;    // kSibSoundDmaPtrShift

	// DinoModule.sibSize sound field, and interrupt1 status bits.
	static constexpr u32 DINO_SIB_TEL_RX_START = 0x06c / 4;
	static constexpr u32 DINO_SIB_TEL_TX_START = 0x070 / 4;
	static constexpr u32 DINO_SIB_TEL_HOLD = 0x07c / 4;

	// Telecom occupies the low half of sibDMA and sibSize.
	static constexpr u32 SIB_TEL_DMA_ONCE = 0x0000'8000;   // kSibTelDmaOnceMask
	static constexpr u32 SIB_TEL_DMA_LOOP = 0x0000'4000;   // kSibTelDmaLoopMask
	static constexpr u32 SIB_TEL_DMA_PTR = 0x0000'3ffc;    // kSibTelDmaPtrMask
	static constexpr u32 SIB_TEL_RX_DMA_EN = 0x0000'0002;  // kSibEnTelRxDmaMask
	static constexpr u32 SIB_TEL_TX_DMA_EN = 0x0000'0001;  // kSibEnTelTxDmaMask
	static constexpr unsigned SIB_TEL_PTR_SHIFT = 2;       // kSibTelDmaPtrShift
	static constexpr u32 SIB_TEL_SIZE = 0x0000'3ffc;       // kSibTelSizeMask

	static constexpr u32 INT1_TEL_DMA_HALF = 0x0010'0000;  // kIntTelDmaHalfMask
	static constexpr u32 INT1_TEL_DMA_END = 0x0008'0000;   // kIntTelDmaEndMask
	static constexpr u32 INT1_TEL_DMA_PTR_INC = 0x0002'0000; // kIntTelDmaPtrIncMask
	static constexpr u32 INT1_TEL_RECEIVE = 0x0000'0200;   // kIntTeleReceiveMask

	static constexpr u32 SIB_SOUND_SIZE = 0x3ffc'0000;     // kSibSoundSizeMask
	static constexpr u32 INT1_SOUND_DMA_HALF = 0x0040'0000; // kIntSoundDmaHalfMask
	static constexpr u32 INT1_SOUND_DMA_END = 0x0020'0000;  // kIntSoundDmaEndMask
	static constexpr u32 INT1_SOUND_DMA_PTR_INC = 0x0004'0000; // kIntSoundDmaPtrIncMask
	static constexpr u32 DINO_VIDEO_HIGH_BUFFER = 0x030 / 4;
	static constexpr u32 DINO_MBUS_CONTROL1 = 0x0e0 / 4;
	static constexpr u32 DINO_MBUS_DMA_START = 0x0e8 / 4;
	static constexpr u32 DINO_MBUS_DMA_LENGTH = 0x0ec / 4;
	static constexpr u32 DINO_MBUS_DMA_COUNT = 0x0f0 / 4;
	static constexpr u32 DINO_MBUS_COMMAND = 0x0f4 / 4;
	static constexpr u32 DINO_MBUS_DATA = 0x0f8 / 4;

	// DinoModule.mbusControl1 status bits, from the SDK's Dino.asm.h.
	static constexpr u32 DINO_MBUS_ENABLED_STATUS = 0x8000'0000; // kMbusEnabledStatusMask
	static constexpr u32 DINO_MBUS_EMPTY_STATUS = 0x4000'0000;   // kMbusEmptyStatusMask
	static constexpr u32 DINO_MBUS_INT_STATUS = 0x2000'0000;     // kMbusIntStatusMask
	static constexpr u32 DINO_MBUS_STATUS =
			DINO_MBUS_ENABLED_STATUS | DINO_MBUS_EMPTY_STATUS | DINO_MBUS_INT_STATUS;

	// interrupt2 Magic Bus bits, from Dino.asm.h.
	static constexpr u32 INT2_MBUS_TRANSMIT = 0x0000'0800; // kIntMbusTransmitMask
	static constexpr u32 INT2_MBUS_EMPTY = 0x0000'0200;    // kIntMbusEmptyMask
	static constexpr u32 INT2_MBUS_COMMAND = 0x0000'0040;  // kIntMbusDetMask
	static constexpr u32 INT2_MBUS_DMA_END = 0x0000'0020;  // kIntMbusDmaEndMask
	static constexpr u32 INT2_MBUS_REQUEST_POS = 0x0000'0008; // kIntMbusIntPosMask
	static constexpr u32 INT2_MBUS_REQUEST_NEG = 0x0000'0004; // kIntMbusIntNegMask
	static constexpr u32 DINO_INTERRUPT1 = 0x100 / 4;
	static constexpr u32 DINO_INTERRUPT2 = 0x104 / 4;
	static constexpr u32 DINO_INTERRUPT3 = 0x108 / 4;
	static constexpr u32 DINO_INTERRUPT4 = 0x10c / 4;
	static constexpr u32 DINO_INTERRUPT5 = 0x110 / 4;
	static constexpr u32 DINO_INTERRUPT6 = 0x114 / 4;
	static constexpr u32 DINO_INTERRUPT1_ENABLE = 0x118 / 4;
	static constexpr u32 DINO_INTERRUPT6_ENABLE = 0x12c / 4;
	static constexpr u32 DINO_RTC_HIGH = 0x140 / 4;
	static constexpr u32 DINO_RTC_LOW = 0x144 / 4;
	static constexpr u32 DINO_ALARM_HIGH = 0x148 / 4;
	static constexpr u32 DINO_ALARM_LOW = 0x14c / 4;
	static constexpr u32 DINO_TIMER_CONTROL = 0x150 / 4;
	static constexpr u32 DINO_PERIODIC_TIMER = 0x154 / 4;
	static constexpr u32 DINO_IO_CONTROL = 0x180 / 4;
	static constexpr u32 DINO_MFIO_DATA_OUTPUT = 0x184 / 4;
	static constexpr u32 DINO_MFIO_DATA_INPUT = 0x18c / 4;

	// Apollo's platform-specific MFIO assignments from Gen2MFS.asm.h.
	// The LCD and charger signals are active high; Magic Bus Vcc-off is
	// active high, so its attached peripheral is powered while this bit is
	// clear.
	static constexpr u32 DINO_MFIO_LCD_POWER = 0x0002'0000;
	static constexpr u32 DINO_MFIO_MBUS_VCC_OFF = 0x0001'0000;
	static constexpr u32 DINO_MFIO_CHARGER_ENABLE = 0x0000'0002;
	static constexpr u32 DINO_MFIO_TELECOM_RING = 0x0000'0001;
	static constexpr u32 INT3_MFIO0_POS = 0x0000'0001;
	static constexpr u32 INT4_MFIO0_NEG = 0x0000'0001;

	// PowerSupplyGen2MFS reads the external-power and battery-cover inputs
	// from these bits: ACAdapterAttached takes powerControl bit 30, and
	// BatteryCoverAttached takes ioControl bit 2 inverted, so the bit is set
	// while the cover is off.  Gen2MFS.asm.h routes the cover switch to IO
	// interrupt 2, whose edges latch in interrupt5.
	static constexpr u32 DINO_POWER_AC_ADAPTER = 0x4000'0000; // kPowerInterruptStatusMask
	static constexpr u32 DINO_IO_COVER_OPEN = 0x0000'0004;
	static constexpr u32 INT5_COVER_OPEN_POS = 0x0000'0200;   // kIntIOInt2PosMask
	static constexpr u32 INT5_COVER_OPEN_NEG = 0x0000'0004;   // kIntIOInt2NegMask
	static constexpr u32 INT5_IR_CARRIER_PIN = 0x0001'0000;   // kIntCarDetPinMask
	static constexpr u32 INT5_IR_CARRIER_POS = 0x0000'8000;   // kIntCarDetPosMask
	static constexpr u32 INT5_IR_CARRIER_NEG = 0x0000'4000;   // kIntCarDetNegMask
	static constexpr u32 DINO_POWER_CONTROL = 0x1c4 / 4;
	static constexpr u32 DINO_INTERRUPT_PENDING_MASK = 0xc000'0000;
	static constexpr u32 DINO_INTERRUPT_LOW_PRIORITY = 0x4000'0000;
	static constexpr u32 DINO_INTERRUPT_GLOBAL_ENABLE = 0x0004'0000;
	static constexpr u32 DINO_ON_BUTTON_POSITIVE = 0x0080'0000;
	static constexpr u32 DINO_ON_BUTTON_NEGATIVE = 0x0040'0000;
	static constexpr u32 DINO_POWER_ON_BUTTON_STATUS = 0x8000'0000;
	static constexpr u32 DINO_POWER_OK_STATUS = 0x2000'0000;
	static constexpr u32 DINO_POWER_STOP_CPU = 0x0000'0010;
	static constexpr u32 DINO_POWER_VCC_ON = 0x0000'0001;
	static constexpr u32 DINO_POWER_WRITE_MASK = 0x0000'ffbf;
	static constexpr u32 GLACIER_IO_DATA_INPUT = 0x00c / 4;
	static constexpr u32 GLACIER_IO_POS_ENABLE = 0x010 / 4;
	static constexpr u32 GLACIER_IO_NEG_ENABLE = 0x014 / 4;
	static constexpr u32 GLACIER_IO_POS_STATUS = 0x018 / 4;
	static constexpr u32 GLACIER_IO_NEG_STATUS = 0x01c / 4;
	static constexpr u32 GLACIER_CONTROL = 0x020 / 4;
	static constexpr u32 VECTOR_PAGE_ROM_OFFSET = 0x0029'6400;
	static constexpr u32 VECTOR_PAGE_RAM_OFFSET = 0x0000'0200;
	static constexpr u32 UART_RX_QUEUE_SIZE = 65'536;

	void memory_map(address_map &map) ATTR_COLD;
	void flash_memory_map(address_map &map) ATTR_COLD;
	void common_memory_map(address_map &map) ATTR_COLD;

	u32 dino_r(offs_t offset, u32 mem_mask = ~0U);
	void dino_w(offs_t offset, u32 data, u32 mem_mask = ~0U);
	u32 glacier1_r(offs_t offset, u32 mem_mask = ~0U);
	void glacier1_w(offs_t offset, u32 data, u32 mem_mask = ~0U);
	u32 glacier2_r(offs_t offset, u32 mem_mask = ~0U);
	void glacier2_w(offs_t offset, u32 data, u32 mem_mask = ~0U);
	u32 vector_page_r(offs_t offset, u32 mem_mask = ~0U);
	void vector_page_w(offs_t offset, u32 data, u32 mem_mask = ~0U);
	template <u32 Base>
	u32 flash_r(offs_t offset, u32 mem_mask = ~0U);
	template <u32 Base>
	void flash_w(offs_t offset, u32 data, u32 mem_mask = ~0U);
	template <unsigned Selector>
	u32 pccard_r(offs_t offset, u32 mem_mask = ~0U);
	template <unsigned Selector>
	void pccard_w(offs_t offset, u32 data, u32 mem_mask = ~0U);
	template <unsigned Slot> void pccard_cd1_w(int state);
	template <unsigned Slot> void pccard_cd2_w(int state);
	template <unsigned Slot> void pccard_bvd1_w(int state);
	template <unsigned Slot> void pccard_bvd2_w(int state);
	template <unsigned Slot> void pccard_wp_w(int state);
	bool pccard_bvd1_level(unsigned slot) const;
	bool pccard_bvd2_level(unsigned slot) const;
	void update_pccard_inputs(unsigned slot);
	u32 glacier_r(unsigned slot, offs_t offset) const;
	void glacier_w(unsigned slot, offs_t offset, u32 data, u32 mem_mask);
	void update_glacier_irq();

	void betty_command(u32 command, bool subframe1);
	void update_betty_irq();
	void set_phone_line(bool connected, bool signal_edge);
	void set_phone_ring(bool ringing, bool signal_edge);
	void restore_inputs();
	u16 touch_adc_value() const;
	u16 main_battery_reading() const;
	u16 backup_battery_reading() const;
	bool magicbus_powered() const;
	bool main_battery_charging() const;
	u32 uart_interrupt_r() const;
	u32 uart_control_r(unsigned channel) const;
	u32 uart_hold_r(unsigned channel);
	void uart_hold_w(unsigned channel, u32 data, u32 mem_mask);
	bool uart_pulsed(unsigned channel) const;
	void irda_received(u8 data);
	void terminal_key(u8 data);
	template <unsigned Channel> void uart_received(u8 data);
	void update_irq();
	void magicbus_set_request(bool asserted);
	void magicbus_command(u16 command);
	void magicbus_deliver_response();
	void magicbus_accept_host_data();
	void update_sib_timers();
	bool sound_dma_running() const;
	void advance_sound_dma();
	s16 microphone_sample(double sample_rate);
	bool telecom_dma_running() const;
	void advance_telecom_dma();
	void update_periodic_timer();
	u64 rtc_ticks() const;
	void persist_rtc();
	void update_rtc_timers();
	TIMER_CALLBACK_MEMBER(periodic_tick);
	TIMER_CALLBACK_MEMBER(rtc_alarm);
	TIMER_CALLBACK_MEMBER(rtc_rollover);
	TIMER_CALLBACK_MEMBER(rtc_persist_tick);
	TIMER_CALLBACK_MEMBER(sib_tick);
	TIMER_CALLBACK_MEMBER(telecom_tick);
	TIMER_CALLBACK_MEMBER(sound_tick);
	TIMER_CALLBACK_MEMBER(magicbus_keyboard_tick);
	TIMER_CALLBACK_MEMBER(battery_charge_tick);
	u32 screen_update(screen_device &screen, bitmap_rgb32 &bitmap, rectangle const &cliprect);

	required_device<r3900_device> m_maincpu;
	required_device<screen_device> m_screen;
	required_device<generic_terminal_device> m_terminal;
	required_device_array<datarover_uart_device, 2> m_uart;
	required_device_array<rs232_port_device, 2> m_rs232;
	required_device<datarover_irda_device> m_irda;
	required_device<dmadac_sound_device> m_dmadac;
	required_device<microphone_device> m_microphone;
	required_device<at_keyboard_device> m_magicbus_keyboard;
	required_device_array<pccard_slot_device, 2> m_pccard;
	optional_device_array<datarover_modem_pccard_device, 2> m_modem_card;
	optional_device_array<fujitsu_29f016a_device, 4> m_flash;
	required_device<nvram_device> m_rtc_nvram;
	required_shared_ptr<u32> m_ram;
	required_region_ptr<u32> m_rom;
	required_ioport m_boot_mode;
	required_ioport m_rtc_resume;
	required_ioport m_battery;
	required_ioport m_power_supply;
	required_ioport m_option_button;
	required_ioport_array<2> m_pccard_battery;
	required_ioport m_touch_x;
	required_ioport m_touch_y;
	required_ioport m_touch_button;
	required_ioport m_power_button;
	required_ioport m_phone_line;
	required_ioport m_phone_ring;
	required_ioport m_magicbus_accessory;
	required_ioport m_irda_carrier;
	required_ioport m_microphone_source;

	std::array<u32, 0x200 / 4> m_dino{};
	std::array<u32, 0x24 / 4> m_glacier1{};
	std::array<u32, 0x24 / 4> m_glacier2{};
	std::array<u16, 16> m_betty{};
	u16 m_betty_pos_pending = 0;
	u16 m_betty_neg_pending = 0;
	bool m_phone_ring_level = false;
	std::array<std::array<u8, UART_RX_QUEUE_SIZE>, 2> m_uart_rx_data{};
	std::array<u32, 2> m_uart_rx_head{};
	std::array<u32, 2> m_uart_rx_count{};
	std::array<bool, 2> m_pccard_cd1{ true, true };
	std::array<bool, 2> m_pccard_cd2{ true, true };
	std::array<bool, 2> m_pccard_ready{ true, true };
	std::array<bool, 2> m_pccard_bvd1{ true, true };
	std::array<bool, 2> m_pccard_bvd2{ true, true };
	std::array<bool, 2> m_pccard_wp{};
	std::array<u32, 5> m_rtc_nvram_data{};
	u64 m_rtc_base = 0;
	u64 m_rtc_origin = 0;
	emu_timer *m_periodic_timer = nullptr;
	emu_timer *m_rtc_alarm_timer = nullptr;
	emu_timer *m_rtc_rollover_timer = nullptr;
	emu_timer *m_rtc_persist_timer = nullptr;
	emu_timer *m_sib_timer = nullptr;
	bool m_sound_dma_half_signalled = false;
	static constexpr u32 MICROPHONE_QUEUE_SIZE = 65'536;
	std::array<s16, MICROPHONE_QUEUE_SIZE> m_microphone_data{};
	u32 m_microphone_head = 0;
	u32 m_microphone_count = 0;
	u32 m_microphone_phase = 0;
	sound_stream *m_microphone_stream = nullptr;
	bool m_telecom_dma_half_signalled = false;
	u32 m_telecom_loopback = 0;
	bool m_magicbus_request = false;
	bool m_magicbus_assigned = false;
	u8 m_magicbus_info_page = 0;
	enum : u8
	{
		MBUS_RESPONSE_NONE,
		MBUS_RESPONSE_ID,
		MBUS_RESPONSE_INFO,
		MBUS_RESPONSE_REQUEST,
		MBUS_RESPONSE_KEYBOARD
	};
	u8 m_magicbus_response = MBUS_RESPONSE_NONE;
	bool m_magicbus_keyboard_read_pending = false;
	bool m_magicbus_host_data_pending = false;
	std::array<u8, 256> m_magicbus_key_data{};
	u16 m_magicbus_key_head = 0;
	u16 m_magicbus_key_count = 0;
	u16 m_main_battery_charge = 0;
	emu_timer *m_telecom_timer = nullptr;
	emu_timer *m_sound_timer = nullptr;
	emu_timer *m_magicbus_keyboard_timer = nullptr;
	emu_timer *m_battery_charge_timer = nullptr;
};


void datarover_state::memory_map(address_map &map)
{
	common_memory_map(map);

	map(0x13c00000, 0x13e963ff).rom().region("maincpu", 0);
	map(0x13e96400, 0x13e965ff).rw(
			FUNC(datarover_state::vector_page_r),
			FUNC(datarover_state::vector_page_w));
	map(0x13e96600, 0x143fffff).rom().region("maincpu", 0x00296600);

	// Architectural reset vector alias.  Only the first word and delay slot
	// are required before the ROM jumps to its normal 0xb3c00000 alias.
	map(0x1fc00000, 0x1fc00007).rom().region("maincpu", 0);
}


void datarover_state::flash_memory_map(address_map &map)
{
	common_memory_map(map);

	map(0x13c00000, 0x13e963ff).rw(
			FUNC(datarover_state::flash_r<0>),
			FUNC(datarover_state::flash_w<0>));
	map(0x13e96400, 0x13e965ff).rw(
			FUNC(datarover_state::vector_page_r),
			FUNC(datarover_state::vector_page_w));
	map(0x13e96600, 0x143fffff).rw(
			FUNC(datarover_state::flash_r<0x00296600>),
			FUNC(datarover_state::flash_w<0x00296600>));
	map(0x1fc00000, 0x1fc00007).rw(
			FUNC(datarover_state::flash_r<0>),
			FUNC(datarover_state::flash_w<0>));
}


void datarover_state::common_memory_map(address_map &map)
{
	map.unmap_value_high();

	map(0x00000000, 0x003fffff).ram().share("ram");

	map(0x08000000, 0x0bffffff).rw(
			FUNC(datarover_state::pccard_r<2>),
			FUNC(datarover_state::pccard_w<2>));
	map(0x0c000000, 0x0fffffff).rw(
			FUNC(datarover_state::pccard_r<3>),
			FUNC(datarover_state::pccard_w<3>));
	map(0x10400000, 0x10400023).rw(
			FUNC(datarover_state::glacier1_r),
			FUNC(datarover_state::glacier1_w));
	map(0x10800000, 0x10800023).rw(
			FUNC(datarover_state::glacier2_r),
			FUNC(datarover_state::glacier2_w));
	map(0x10c00000, 0x10c001ff).rw(
			FUNC(datarover_state::dino_r),
			FUNC(datarover_state::dino_w));
	map(0x24000000, 0x27ffffff).rw(
			FUNC(datarover_state::pccard_r<0>),
			FUNC(datarover_state::pccard_w<0>));
	map(0x28000000, 0x2bffffff).rw(
			FUNC(datarover_state::pccard_r<1>),
			FUNC(datarover_state::pccard_w<1>));
}


u32 datarover_state::vector_page_r(offs_t offset, u32 mem_mask)
{
	if (BIT(m_dino[0], 25))
		return m_ram[(VECTOR_PAGE_RAM_OFFSET / 4) + offset];
	if (m_flash[0])
		return flash_r<VECTOR_PAGE_ROM_OFFSET>(offset, mem_mask);

	return m_rom[(VECTOR_PAGE_ROM_OFFSET / 4) + offset];
}


void datarover_state::vector_page_w(offs_t offset, u32 data, u32 mem_mask)
{
	// SetupVectorDispatching copies this ROM page to RAM at 0x200, then
	// enables Dino address-remap region 1 before patching its dispatch stub.
	if (BIT(m_dino[0], 25))
		COMBINE_DATA(&m_ram[(VECTOR_PAGE_RAM_OFFSET / 4) + offset]);
	else if (m_flash[0])
		flash_w<VECTOR_PAGE_ROM_OFFSET>(offset, data, mem_mask);
}


template <u32 Base>
u32 datarover_state::flash_r(offs_t offset, u32 mem_mask)
{
	u32 result = 0xffff'ffff;
	u32 const chip_offset = (Base / 4) + offset;
	for (unsigned lane = 0; lane < m_flash.size(); ++lane)
	{
		unsigned const shift = 24 - (lane * 8);
		u32 const lane_mask = 0xffU << shift;
		if (mem_mask & lane_mask)
			result = (result & ~lane_mask) | (u32(m_flash[lane]->read(chip_offset)) << shift);
	}
	return result;
}


template <u32 Base>
void datarover_state::flash_w(offs_t offset, u32 data, u32 mem_mask)
{
	u32 const chip_offset = (Base / 4) + offset;
	for (unsigned lane = 0; lane < m_flash.size(); ++lane)
	{
		unsigned const shift = 24 - (lane * 8);
		u32 const lane_mask = 0xffU << shift;
		if (mem_mask & lane_mask)
			m_flash[lane]->write(chip_offset, u8(data >> shift));
	}
}


template <unsigned Selector>
u32 datarover_state::pccard_r(offs_t offset, u32 mem_mask)
{
	static constexpr unsigned Slot = Selector & 1;
	static constexpr bool Attribute = BIT(Selector, 1);
	u32 result = 0xffff'ffff;
	if (ACCESSING_BITS_16_31)
	{
		u16 const value = Attribute
				? m_pccard[Slot]->read_reg_swap(offset * 2, u16(mem_mask >> 16))
				: m_pccard[Slot]->read_memory_swap(offset * 2, u16(mem_mask >> 16));
		result = (result & 0x0000'ffff) | (u32(value) << 16);
	}
	if (ACCESSING_BITS_0_15)
	{
		u16 const value = Attribute
				? m_pccard[Slot]->read_reg_swap((offset * 2) + 1, u16(mem_mask))
				: m_pccard[Slot]->read_memory_swap((offset * 2) + 1, u16(mem_mask));
		result = (result & 0xffff'0000) | value;
	}
	return result;
}


template <unsigned Selector>
void datarover_state::pccard_w(offs_t offset, u32 data, u32 mem_mask)
{
	static constexpr unsigned Slot = Selector & 1;
	static constexpr bool Attribute = BIT(Selector, 1);
	if (ACCESSING_BITS_16_31)
	{
		if constexpr (Attribute)
			m_pccard[Slot]->write_reg_swap(offset * 2, u16(data >> 16), u16(mem_mask >> 16));
		else
			m_pccard[Slot]->write_memory_swap(offset * 2, u16(data >> 16), u16(mem_mask >> 16));
	}
	if (ACCESSING_BITS_0_15)
	{
		if constexpr (Attribute)
			m_pccard[Slot]->write_reg_swap((offset * 2) + 1, u16(data), u16(mem_mask));
		else
			m_pccard[Slot]->write_memory_swap((offset * 2) + 1, u16(data), u16(mem_mask));
	}
}


void datarover_state::update_pccard_inputs(unsigned slot)
{
	std::array<u32, 0x24 / 4> &glacier = slot ? m_glacier2 : m_glacier1;
	u16 const old_value = glacier[GLACIER_IO_DATA_INPUT] >> 16;
	bool const present = !m_pccard_cd1[slot] && !m_pccard_cd2[slot];
	u16 value = 0;
	if (m_pccard_cd1[slot])
		value |= 0x0400;
	if (m_pccard_cd2[slot])
		value |= 0x0800;
	if (present)
	{
		value |= 0x0300; // five-volt card voltage-sense coding
		if (m_pccard_ready[slot])
			value |= 0x0004; // READY, or active-low IREQ in I/O mode
		if (pccard_bvd2_level(slot))
			value |= 0x0002;
		if (m_pccard_wp[slot])
			value |= 0x0008;
	}

	glacier[GLACIER_IO_DATA_INPUT] =
			(glacier[GLACIER_IO_DATA_INPUT] & 0x0000'ffff)
			| (u32(value) << 16);

	// Glacier latches both edges even while their interrupt enables are off.
	// This preserves an insertion that occurs while Magic Cap initializes the
	// slot and delivers it as soon as the driver enables the corresponding
	// edge interrupt.
	u16 const rising = ~old_value & value;
	u16 const falling = old_value & ~value;
	glacier[GLACIER_IO_POS_STATUS] |= u32(rising) << 16;
	glacier[GLACIER_IO_NEG_STATUS] |= u32(falling) << 16;

	if (m_periodic_timer)
	{
		update_glacier_irq();
		update_irq();
	}
}


bool datarover_state::pccard_bvd1_level(unsigned slot) const
{
	unsigned const battery = m_pccard_battery[slot]->read();
	return m_pccard_bvd1[slot] && (battery == 3 || battery == 1);
}


bool datarover_state::pccard_bvd2_level(unsigned slot) const
{
	return m_pccard_bvd2[slot] && m_pccard_battery[slot]->read() == 3;
}


template <unsigned Slot>
void datarover_state::pccard_cd1_w(int state)
{
	m_pccard_cd1[Slot] = bool(state);
	update_pccard_inputs(Slot);
}


template <unsigned Slot>
void datarover_state::pccard_cd2_w(int state)
{
	m_pccard_cd2[Slot] = bool(state);
	update_pccard_inputs(Slot);
}


template <unsigned Slot>
void datarover_state::pccard_bvd1_w(int state)
{
	// Apollo routes a memory card's BVD1 through Dino IO bits 1/0.  In PC
	// Card I/O mode the same physical pin becomes active-low IREQ, sampled
	// on Glacier's READY/IREQ input by the serial-modem card path.
	m_pccard_bvd1[Slot] = bool(state);
	m_pccard_ready[Slot] = bool(state);
	update_pccard_inputs(Slot);
}


template <unsigned Slot>
void datarover_state::pccard_bvd2_w(int state)
{
	m_pccard_bvd2[Slot] = bool(state);
	update_pccard_inputs(Slot);
}


template <unsigned Slot>
void datarover_state::pccard_wp_w(int state)
{
	m_pccard_wp[Slot] = bool(state);
	update_pccard_inputs(Slot);
}


u32 datarover_state::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, rectangle const &cliprect)
{
	static constexpr std::array<u8, 4> LEVEL{ 0xff, 0xaa, 0x55, 0x00 };
	static constexpr u32 FALLBACK_BASE = 0x003f'6a00;
	static constexpr u32 FRAME_BYTES = 480 * 320 / 4;

	(void)screen;

	// Apollo drives the panel supply from MFIO 17.  The framebuffer remains
	// intact while the rail is off, just as it does through normal suspend.
	if (!(m_dino[DINO_MFIO_DATA_OUTPUT] & DINO_MFIO_LCD_POWER))
	{
		bitmap.fill(rgb_t::black(), cliprect);
		return 0;
	}

	u32 base = m_dino[DINO_VIDEO_HIGH_BUFFER] & 0xffff'fff0;
	if (base > (0x0040'0000 - FRAME_BYTES))
		base = FALLBACK_BASE;

	address_space &space = m_maincpu->space(AS_PROGRAM);
	for (int y = cliprect.min_y; y <= cliprect.max_y; ++y)
	{
		u32 pixels = 0;
		for (int x = cliprect.min_x; x <= cliprect.max_x; ++x)
		{
			if (x == cliprect.min_x || !(x & 15))
				pixels = space.read_dword(base + (y * 120) + ((x >> 4) * 4));

			u8 const level = LEVEL[BIT(pixels, 30 - (2 * (x & 15)), 2)];
			bitmap.pix(y, x) = rgb_t(level, level, level);
		}
	}

	return 0;
}


u32 datarover_state::uart_control_r(unsigned channel) const
{
	u32 const control =
			m_dino[channel ? DINO_UART_B_CONTROL1 : DINO_UART_A_CONTROL1];
	u32 result = (control & ~DINO_UART_STATUS) | DINO_UART_EMPTY_STATUS;
	if (BIT(control, 0))
		result |= DINO_UART_ENABLED_STATUS;
	if (m_uart_rx_count[channel])
		result |= DINO_UART_RX_HOLD_FULL;
	if (m_uart_rx_count[channel] > 1)
		result |= DINO_UART_PRX_HOLD_FULL;

	return result;
}


u32 datarover_state::uart_hold_r(unsigned channel)
{
	u8 data = 0;
	if (m_uart_rx_count[channel])
	{
		data = m_uart_rx_data[channel][m_uart_rx_head[channel]];
		m_uart_rx_head[channel] =
				(m_uart_rx_head[channel] + 1) % UART_RX_QUEUE_SIZE;
		--m_uart_rx_count[channel];
	}
	update_irq();
	return data;
}


void datarover_state::uart_hold_w(unsigned channel, u32 data, u32 mem_mask)
{
	if (ACCESSING_BITS_0_7)
	{
		u8 const character = data;
		if (uart_pulsed(channel))
		{
			m_irda->transmit(character);
		}
		else
		{
			m_terminal->write(character);
			if (m_rs232[channel]->get_card_device())
				m_uart[channel]->transmit(character);
		}
		m_dino[DINO_INTERRUPT2] |= channel ? 0x0001'0000 : 0x0400'0000;
		logerror(
				"UART%c%s TX: %02x %c\n",
				'A' + channel,
				uart_pulsed(channel) ? " IrDA" : "",
				character,
				(character >= 0x20 && character < 0x7f) ? character : '.');
		update_irq();
	}
}


bool datarover_state::uart_pulsed(unsigned channel) const
{
	u32 const control =
			m_dino[channel ? DINO_UART_B_CONTROL1 : DINO_UART_A_CONTROL1];
	return bool(control & DINO_UART_PULSED_MODE);
}


void datarover_state::irda_received(u8 data)
{
	for (unsigned channel = 0; channel < 2; ++channel)
	{
		if (uart_pulsed(channel))
		{
			if (channel)
				uart_received<1>(data);
			else
				uart_received<0>(data);
			return;
		}
	}

	logerror("IrDA RX dropped while neither UART is in pulsed mode: %02x\n", data);
}


void datarover_state::terminal_key(u8 data)
{
	// The IDT monitor uses UART A by default.
	uart_received<0>(data);
}


template <unsigned Channel>
void datarover_state::uart_received(u8 data)
{
	if (m_uart_rx_count[Channel] < UART_RX_QUEUE_SIZE)
	{
		unsigned const tail =
				(m_uart_rx_head[Channel] + m_uart_rx_count[Channel])
				% UART_RX_QUEUE_SIZE;
		m_uart_rx_data[Channel][tail] = data;
		++m_uart_rx_count[Channel];
	}
	else
	{
		// Preserve an actual overrun once the emulator-side ingress queue
		// behind Dino's two documented holding stages is exhausted.
		m_dino[DINO_INTERRUPT2] |=
				Channel ? 0x0010'0000 : 0x4000'0000;
		logerror("UART%c RX overrun\n", 'A' + Channel);
	}
	update_irq();
}


void datarover_state::betty_command(u32 command, bool subframe1)
{
	unsigned const reg = BIT(command, 27, 4);
	bool const write = BIT(command, 26);

	if (write && reg != 12)
	{
		u16 const old_value = m_betty[reg];
		switch (reg)
		{
		case 0:
			// Betty IOData mixes writable outputs with sampled inputs.  Keep
			// the phone-line and touchscreen inputs under external control.
			m_betty[reg] = (u16(command) & ~0x1100) | (m_betty[reg] & 0x1100);
			break;

		case 4:
		{
			// Betty reports latched GPIO edges here.  The SIB interrupt
			// handler reads the mask, then acknowledges it by writing the
			// same set bits back.
			u16 const clear = u16(command);
			m_betty[4] &= ~clear;
			m_betty_pos_pending &= ~clear;
			m_betty_neg_pending &= ~clear;
			break;
		}

		default:
			m_betty[reg] = u16(command);
			break;
		}

		if (reg == 10)
		{
			m_betty[11] = 0x8000 | ((touch_adc_value() & 0x03ff) << 5);
		}
		if (reg == 8 && BIT(old_value ^ m_betty[reg], 14))
		{
			// Betty's sound-input enable starts/stops Dino receive DMA as a
			// hardware side effect.  SibCmdStartSoundIn sets this bit after
			// programming the receive address and size; unlike sound output,
			// the ROM never writes kSibEnSoundRxDmaMask itself.
			if (BIT(m_betty[reg], 14))
			{
				m_dino[DINO_SIB_DMA] &= ~SIB_SOUND_DMA_PTR;
				m_dino[DINO_SIB_DMA] |= SIB_SOUND_RX_DMA_EN;
				m_sound_dma_half_signalled = false;
				m_microphone_head = 0;
				m_microphone_count = 0;
				m_microphone_phase = 0;
			}
			else
			{
				m_dino[DINO_SIB_DMA] &= ~SIB_SOUND_RX_DMA_EN;
			}
		}
		if (reg == 2)
			m_betty_pos_pending &= m_betty[2];
		else if (reg == 3)
			m_betty_neg_pending &= m_betty[3];
		update_betty_irq();
	}

	// Betty supplies valid audio slots once microphone input is enabled.
	// SibCmdStartSoundIn waits for this Dino status bit before completing;
	// without it the SIB command queue remains occupied and StopSoundIn can
	// never run.
	m_dino[subframe1 ? DINO_SIB_SF1_STATUS : DINO_SIB_SF0_STATUS] =
			m_betty[reg]
			| (BIT(m_betty[8], 14) ? SIB_STATUS_AUDIO_VALID : 0);
	m_dino[DINO_INTERRUPT1] |= subframe1 ? 0x00000080 : 0x00000100;
	update_irq();
}


void datarover_state::update_betty_irq()
{
	// Betty's IRQ output is level-like: latched edges gated by the enable
	// registers.  Reflect an enabled pending edge into Dino's interrupt
	// status so an edge that latched while the OS had the enables masked
	// (it masks them around every ADC macro) is presented as soon as the
	// mask is restored, instead of being lost.
	bool const pos = bool(m_betty_pos_pending & m_betty[2]);
	bool const neg = bool(m_betty_neg_pending & m_betty[3]);
	if (pos)
		m_dino[DINO_INTERRUPT1] |= 0x0000'0040;
	if (neg)
		m_dino[DINO_INTERRUPT1] |= 0x0000'0020;
	if (pos || neg)
		m_dino[DINO_SIB_CONTROL] |= 0x8000'0000;
	else
		m_dino[DINO_SIB_CONTROL] &= ~0x8000'0000;
	update_irq();
}


void datarover_state::set_phone_line(bool connected, bool signal_edge)
{
	static constexpr u16 PHONE_LINE_MASK = 0x0100;
	bool const was_connected = bool(m_betty[0] & PHONE_LINE_MASK);
	if (was_connected == connected)
		return;

	if (connected)
		m_betty[0] |= PHONE_LINE_MASK;
	else
		m_betty[0] &= ~PHONE_LINE_MASK;

	if (signal_edge)
	{
		u16 const enabled = connected ? m_betty[2] : m_betty[3];
		if (enabled & PHONE_LINE_MASK)
		{
			if (connected)
				m_betty_pos_pending |= PHONE_LINE_MASK;
			else
				m_betty_neg_pending |= PHONE_LINE_MASK;
			m_betty[4] |= PHONE_LINE_MASK;
			m_dino[DINO_INTERRUPT1] |= connected ? 0x0000'0040 : 0x0000'0020;
		}
	}

	update_betty_irq();
	update_irq();
}

void datarover_state::set_phone_ring(bool ringing, bool signal_edge)
{
	if (m_phone_ring_level == ringing)
		return;

	m_phone_ring_level = ringing;
	if (signal_edge)
		m_dino[ringing ? DINO_INTERRUPT3 : DINO_INTERRUPT4] |=
				ringing ? INT3_MFIO0_POS : INT4_MFIO0_NEG;
	update_irq();
}


void datarover_state::restore_inputs()
{
	set_phone_line(bool(m_phone_line->read()), true);
	set_phone_ring(bool(m_phone_ring->read()), true);
	for (unsigned slot = 0; slot < m_modem_card.size(); ++slot)
	{
		if (m_modem_card[slot])
		{
			bool const saved_empty =
					m_pccard_cd1[slot] || m_pccard_cd2[slot];
			m_modem_card[slot]->restore_state(saved_empty);
		}
	}
}


u16 datarover_state::main_battery_reading() const
{
	// Full charge reads at the calibration record's full point, so the OS
	// reports 100%.  "Low" sits under the 320-count warning threshold but
	// above empty; "empty" is below the 80-count floor, which is what the OS
	// treats as a flat cell.
	u16 base;
	switch (BIT(m_battery->read(), 0, 2))
	{
	case 1:  base = 200; break; // between empty (80) and the 320 warning point
	case 2:  base = 60; break;  // below empty
	default: base = 800; break; // the record's full point, so the OS reports 100%
	}
	return std::min<u16>(base + m_main_battery_charge, 800);
}


u16 datarover_state::backup_battery_reading() const
{
	// The backup cell's thresholds are 400 empty, 816 low.  A healthy reading
	// has to clear 816: the driver used to answer 340 here, below even the
	// empty point, so every boot posted "your backup battery is almost out of
	// power".
	switch (BIT(m_battery->read(), 2, 2))
	{
	case 1:  return 700;   // between empty (400) and the 816 warning point
	case 2:  return 300;   // below empty; the OS posts "completely out of power"
	default: return 1000;  // clears the warning point with margin
	}
}


u16 datarover_state::touch_adc_value() const
{
	// MainBatteryServer_InitAtoDChannel and its backup counterpart select
	// Betty ADC inputs 24 and 28.  BatteryServer_CalculateLevel turns a
	// reading into a percentage between the "empty" and "full" fields taken
	// from the Apollo calibration record the ROM picks for Betty revision 1,
	// and warns below the "low" field:
	//
	//              empty   low    full
	//   main  (24)    80    320    800   record 0x13e96dc0
	//   backup(28)   400    816   1600   record 0x13e96e20
	//
	// The backup channel's full point sits above the 10-bit reading the ADC
	// returns, so a healthy cell reads mid-scale rather than 100%.
	switch (m_betty[10] & 0x001d)
	{
	case 0x18:
		return main_battery_reading();

	case 0x1c:
		return backup_battery_reading();
	}

	// Apollo's touch macro selects the electrode arrangement through Betty's
	// TouchCfg register, then samples six ADC values.  The middle pair are X
	// and Y; the other four are pressure/contact checks.
	switch (m_betty[9])
	{
	case 0x0a12:
		return 136 + ((u32(m_touch_x->read()) * (959 - 136)) / 0xffff);

	case 0x0a48:
		return 212 + ((u32(m_touch_y->read()) * (885 - 212)) / 0xffff);

	default:
		return m_touch_button->read() ? 100 : 0;
	}
}


INPUT_CHANGED_MEMBER(datarover_state::touch_changed)
{
	static constexpr u16 TOUCH_MASK = 0x1000;

	// Latch the edge unconditionally; the enable registers only gate the
	// IRQ (see update_betty_irq).  Gating the latch itself dropped edges
	// that arrived while the OS had the enables masked around an ADC
	// macro, leaving Magic Cap waiting forever for a pen transition.
	if (newval)
	{
		m_betty[0] |= TOUCH_MASK;
		m_betty_pos_pending |= TOUCH_MASK;
		m_betty[4] |= TOUCH_MASK;
	}
	else
	{
		m_betty[0] &= ~TOUCH_MASK;
		m_betty_neg_pending |= TOUCH_MASK;
		m_betty[4] |= TOUCH_MASK;
	}

	update_betty_irq();
}


INPUT_CHANGED_MEMBER(datarover_state::option_changed)
{
	// Dino samples the physical button as an active-low input on IO control
	// bit 3.  Interrupt 5 presents separate falling- and rising-edge sources.
	m_dino[DINO_INTERRUPT5] |= newval ? 0x0000'0008 : 0x0000'0400;
	update_irq();
}


INPUT_CHANGED_MEMBER(datarover_state::pccard_battery_changed)
{
	unsigned const slot = unsigned(param);
	auto const bvd1_for = [](u32 battery) {
		return battery == 3 || battery == 1;
	};
	bool const old_bvd1 = m_pccard_bvd1[slot] && bvd1_for(oldval);
	bool const new_bvd1 = m_pccard_bvd1[slot] && bvd1_for(newval);
	if (old_bvd1 != new_bvd1)
	{
		// The BVD1 pins are Dino IO inputs 1/0, with corresponding
		// positive- and negative-edge sources in interrupt bank 5.
		if (new_bvd1)
			m_dino[DINO_INTERRUPT5] |= slot ? 0x0000'0080 : 0x0000'0100;
		else
			m_dino[DINO_INTERRUPT5] |= slot ? 0x0000'0001 : 0x0000'0002;
	}
	update_pccard_inputs(slot);
	update_irq();
}


INPUT_CHANGED_MEMBER(datarover_state::power_changed)
{
	if (newval)
	{
		m_dino[DINO_INTERRUPT5] |= DINO_ON_BUTTON_POSITIVE;
		if (m_maincpu->suspended(SUSPEND_REASON_HALT))
		{
			// The on-button releases StopCpu for both doze (VCC retained)
			// and power-down (VCC removed).  Only the latter resets Betty
			// and the SIB state on its switched peripheral rail.
			if (!BIT(m_dino[DINO_POWER_CONTROL], 0))
			{
				m_betty.fill(0);
				m_betty_pos_pending = 0;
				m_betty_neg_pending = 0;
				m_betty[12] = 0x1002;
				m_dino[DINO_SIB_CONTROL] = 0;
				m_dino[DINO_SIB_SF0_STATUS] = 0;
				m_dino[DINO_SIB_SF1_STATUS] = 0;
				m_dino[DINO_INTERRUPT1] &= ~0x0000'05e0;
				m_sib_timer->reset();
				m_telecom_timer->reset();
				m_telecom_dma_half_signalled = false;
				m_telecom_loopback = 0;
				m_sound_timer->reset();
			}
			m_dino[DINO_POWER_CONTROL] |= DINO_POWER_VCC_ON;
			m_dino[DINO_POWER_CONTROL] &= ~DINO_POWER_STOP_CPU;
			m_maincpu->resume(SUSPEND_REASON_HALT);
		}
	}
	else
	{
		m_dino[DINO_INTERRUPT5] |= DINO_ON_BUTTON_NEGATIVE;
	}

	update_irq();
}


INPUT_CHANGED_MEMBER(datarover_state::phone_line_changed)
{
	set_phone_line(bool(newval), true);
}

INPUT_CHANGED_MEMBER(datarover_state::phone_ring_changed)
{
	set_phone_ring(bool(newval), true);
}


INPUT_CHANGED_MEMBER(datarover_state::main_battery_changed)
{
	// A machine-configuration change chooses a new synthetic starting charge.
	// From there the charger model advances it over emulated time.
	m_main_battery_charge = 0;
}


INPUT_CHANGED_MEMBER(datarover_state::battery_cover_changed)
{
	// Removing the cover is the positive edge on IO interrupt 2; refitting it
	// is the negative one.  MainBatteryCoverPositive and its negative
	// counterpart are the ROM's handlers.
	m_dino[DINO_INTERRUPT5] |= newval ? INT5_COVER_OPEN_POS : INT5_COVER_OPEN_NEG;
	update_irq();
}


INPUT_CHANGED_MEMBER(datarover_state::irda_carrier_changed)
{
	// The DataRover's IrDA transceiver drives Dino's carrier-detect pin.
	// SerialServerDinoIrDA enables its positive-edge interrupt while waiting
	// for a peer, using the edge to wake the IrLAP daemon before bytes arrive.
	m_dino[DINO_INTERRUPT5] |=
			newval ? INT5_IR_CARRIER_POS : INT5_IR_CARRIER_NEG;
	update_irq();
}


u32 datarover_state::uart_interrupt_r() const
{
	u32 result = m_dino[DINO_INTERRUPT2];
	// The early IDT monitor polls the UART's empty/ready levels without
	// enabling the production interrupt-driven serial server.
	if (!m_boot_mode->read())
		result |= 0x0501'4000;
	if (m_uart_rx_count[0])
		result |= 0x80000000;
	if (m_uart_rx_count[1])
		result |= 0x00200000;

	return result;
}

void datarover_state::update_irq()
{
	bool pending = false;

	if (m_dino[DINO_INTERRUPT6_ENABLE] & DINO_INTERRUPT_GLOBAL_ENABLE)
	{
		for (unsigned bank = 0; bank < 5; ++bank)
		{
			u32 const status = (bank == 1)
					? uart_interrupt_r()
					: m_dino[DINO_INTERRUPT1 + bank];
			pending |= bool(status & m_dino[DINO_INTERRUPT1_ENABLE + bank]);
		}
	}

	// Interrupt 6 is Dino's read-only summary bank.  DeepDoze polls its
	// high/low pending bits with CPU interrupts masked, so asserting only
	// the R3900 input line leaves the ROM's wake loop spinning forever.
	m_dino[DINO_INTERRUPT6] &= ~DINO_INTERRUPT_PENDING_MASK;
	if (pending)
		m_dino[DINO_INTERRUPT6] |= DINO_INTERRUPT_LOW_PRIORITY;

	// StopCpu is released by an enabled interrupt while VCC remains on.
	// Power-down keeps VCC clear and can only be resumed by the on-button
	// path above.
	if (pending
			&& BIT(m_dino[DINO_POWER_CONTROL], 0)
			&& m_maincpu->suspended(SUSPEND_REASON_HALT))
	{
		m_dino[DINO_POWER_CONTROL] &= ~DINO_POWER_STOP_CPU;
		m_maincpu->resume(SUSPEND_REASON_HALT);
	}

	// Dino's general interrupt is IP4 (Cause bit 12), MIPS input line 2.
	m_maincpu->set_input_line(2, pending ? ASSERT_LINE : CLEAR_LINE);
}


bool datarover_state::sound_dma_running() const
{
	// Sound DMA needs the SIB enabled, its sound channel enabled, and the
	// direction enable the ROM sets last.
	return BIT(m_dino[DINO_SIB_CONTROL], 0)
			&& BIT(m_dino[DINO_SIB_CONTROL], 4)
			&& (m_dino[DINO_SIB_DMA]
					& (SIB_SOUND_RX_DMA_EN | SIB_SOUND_TX_DMA_EN));
}


bool datarover_state::magicbus_powered() const
{
	return !(m_dino[DINO_MFIO_DATA_OUTPUT] & DINO_MFIO_MBUS_VCC_OFF);
}


bool datarover_state::main_battery_charging() const
{
	return BIT(m_power_supply->read(), 0)
			&& !BIT(m_power_supply->read(), 1)
			&& (m_dino[DINO_MFIO_DATA_OUTPUT] & DINO_MFIO_CHARGER_ENABLE)
			&& main_battery_reading() < 800;
}


void datarover_state::advance_sound_dma()
{
	// The sound size and DMA pointer fields both count 32-bit words, and the
	// ROM programs the size as the last valid index rather than a count.
	u32 const words = ((m_dino[DINO_SIB_SIZE] & SIB_SOUND_SIZE) >> SIB_SOUND_PTR_SHIFT) + 1;
	u32 ptr = (m_dino[DINO_SIB_DMA] & SIB_SOUND_DMA_PTR) >> SIB_SOUND_PTR_SHIFT;

	// The ROM hands Dino start addresses with the segment bits still set, so
	// mask them to physical DRAM addresses the way the SIB bus master would.
	if (m_dino[DINO_SIB_DMA] & SIB_SOUND_RX_DMA_EN)
	{
		if (!m_microphone_source->read())
			m_microphone_stream->update();

		unsigned const divisor = BIT(m_dino[DINO_SIB_CONTROL], 8, 7) + 1;
		double const sample_rate = 9'216'000.0 / (32.0 * divisor);
		u16 const first = u16(microphone_sample(sample_rate));
		u16 const second = u16(microphone_sample(sample_rate));
		u32 const word = (u32(first) << 16) | second;
		u32 const base = m_dino[DINO_SIB_SOUND_RX_START] & 0x1fff'fffc;
		m_maincpu->space(AS_PROGRAM).write_dword(base + ptr * 4, word);
	}

	if ((m_dino[DINO_SIB_DMA] & SIB_SOUND_TX_DMA_EN)
			&& BIT(m_betty[8], 15))
	{
		u32 const base = m_dino[DINO_SIB_SOUND_TX_START] & 0x1fff'fffc;
		u32 const word =
				m_maincpu->space(AS_PROGRAM).read_dword(base + ptr * 4);
		// One 32-bit slot carries two signed 16-bit samples, most significant
		// first, matching the unbuffered hold register.
		std::array<s16, 2> samples{ s16(word >> 16), s16(word) };
		m_dmadac->transfer(0, 1, 1, samples.size(), samples.data());
	}

	++ptr;
	m_dino[DINO_INTERRUPT1] |= INT1_SOUND_DMA_PTR_INC;

	if (!m_sound_dma_half_signalled && ptr >= words / 2)
	{
		m_sound_dma_half_signalled = true;
		m_dino[DINO_INTERRUPT1] |= INT1_SOUND_DMA_HALF;
	}

	if (ptr >= words)
	{
		m_dino[DINO_INTERRUPT1] |= INT1_SOUND_DMA_END;
		m_sound_dma_half_signalled = false;
		ptr = 0;

		// The normal mode is a continuously serviced two-half ring.  The ROM
		// fills each half from its half/full interrupt handlers without setting
		// kSibSoundDmaLoopMask.  Only an explicitly requested one-shot transfer
		// stops at the end.
		if (m_dino[DINO_SIB_DMA] & SIB_SOUND_DMA_ONCE)
			m_dino[DINO_SIB_DMA] &=
					~(SIB_SOUND_RX_DMA_EN | SIB_SOUND_TX_DMA_EN);
	}

	// The pointer field is hardware-owned; SibServerSyncSoundOutDma reads it
	// back to find how far playback has progressed.
	m_dino[DINO_SIB_DMA] = (m_dino[DINO_SIB_DMA] & ~SIB_SOUND_DMA_PTR)
			| ((ptr << SIB_SOUND_PTR_SHIFT) & SIB_SOUND_DMA_PTR);

	update_irq();
}


s16 datarover_state::microphone_sample(double sample_rate)
{
	switch (m_microphone_source->read())
	{
	case 1:
	{
		// A stable full-scale-safe source for regression and sound-stamp demos.
		static constexpr double TWO_PI = 6.28318530717958647692;
		double const angle =
				double(m_microphone_phase) * (TWO_PI / 4'294'967'296.0);
		s16 const sample = s16(std::lround(std::sin(angle) * 12'000.0));
		u32 const step = u32(std::lround(
				(1'000.0 * 4'294'967'296.0) / sample_rate));
		m_microphone_phase += step;
		return sample;
	}

	case 2:
		return 0;

	default:
		if (m_microphone_count)
		{
			s16 const sample = m_microphone_data[m_microphone_head];
			m_microphone_head =
					(m_microphone_head + 1) % MICROPHONE_QUEUE_SIZE;
			--m_microphone_count;
			return sample;
		}
		return 0;
	}
}


void datarover_state::sound_stream_update(sound_stream &stream)
{
	for (int index = 0; index < stream.samples(); ++index)
	{
		s16 const sample = s16(std::lround(std::clamp(
				stream.get(0, index), -1.0F, 1.0F) * 32'767.0F));
		if (m_microphone_count == MICROPHONE_QUEUE_SIZE)
			m_microphone_head =
					(m_microphone_head + 1) % MICROPHONE_QUEUE_SIZE;
		else
			++m_microphone_count;
		m_microphone_data[
				(m_microphone_head + m_microphone_count - 1)
				% MICROPHONE_QUEUE_SIZE] = sample;
	}
	stream.fill(0, 0.0F);
}


bool datarover_state::telecom_dma_running() const
{
	// kSibEnableTel gates the channel; either direction's DMA enable starts
	// the transfer, and the two share one pointer in sibDMA.
	return BIT(m_dino[DINO_SIB_CONTROL], 0)
			&& BIT(m_dino[DINO_SIB_CONTROL], 5)
			&& (m_dino[DINO_SIB_DMA] & (SIB_TEL_TX_DMA_EN | SIB_TEL_RX_DMA_EN));
}


void datarover_state::advance_telecom_dma()
{
	u32 const words = ((m_dino[DINO_SIB_SIZE] & SIB_TEL_SIZE) >> SIB_TEL_PTR_SHIFT) + 1;
	u32 ptr = (m_dino[DINO_SIB_DMA] & SIB_TEL_DMA_PTR) >> SIB_TEL_PTR_SHIFT;
	address_space &space = m_maincpu->space(AS_PROGRAM);

	// Transmit first: the DAA and phone line are not modelled, so the samples
	// the software modem produces are consumed at the programmed rate.  With
	// kSibLoopModeMask set the SIB feeds them straight back, which is what the
	// modem's own loopback diagnostics expect.
	if (m_dino[DINO_SIB_DMA] & SIB_TEL_TX_DMA_EN)
	{
		u32 const base = m_dino[DINO_SIB_TEL_TX_START] & 0x1fff'fffc;
		m_telecom_loopback = space.read_dword(base + ptr * 4);
	}

	if (m_dino[DINO_SIB_DMA] & SIB_TEL_RX_DMA_EN)
	{
		u32 const base = m_dino[DINO_SIB_TEL_RX_START] & 0x1fff'fffc;
		bool const loopback = BIT(m_dino[DINO_SIB_CONTROL], 3);
		space.write_dword(base + ptr * 4, loopback ? m_telecom_loopback : 0);
	}

	++ptr;
	m_dino[DINO_INTERRUPT1] |= INT1_TEL_DMA_PTR_INC;

	if (!m_telecom_dma_half_signalled && ptr >= words / 2)
	{
		m_telecom_dma_half_signalled = true;
		m_dino[DINO_INTERRUPT1] |= INT1_TEL_DMA_HALF;
	}

	if (ptr >= words)
	{
		m_dino[DINO_INTERRUPT1] |= INT1_TEL_DMA_END;
		m_telecom_dma_half_signalled = false;
		ptr = 0;

		// The software modem uses the buffer as a continuously serviced
		// two-half ring: SibCmdStartTelecom enables RX/TX without setting
		// kSibTelDmaOnceMask or kSibTelDmaLoopMask, and its half/full
		// interrupt handlers process each 48-sample half in place.  Only an
		// explicitly requested one-shot transfer stops at the end.
		if (m_dino[DINO_SIB_DMA] & SIB_TEL_DMA_ONCE)
			m_dino[DINO_SIB_DMA] &= ~(SIB_TEL_TX_DMA_EN | SIB_TEL_RX_DMA_EN);
	}

	m_dino[DINO_SIB_DMA] = (m_dino[DINO_SIB_DMA] & ~SIB_TEL_DMA_PTR)
			| ((ptr << SIB_TEL_PTR_SHIFT) & SIB_TEL_DMA_PTR);

	update_irq();
}


void datarover_state::update_sib_timers()
{
	bool const sib_enabled = BIT(m_dino[DINO_SIB_CONTROL], 0);
	if (sib_enabled)
		m_sib_timer->adjust(attotime::from_msec(1), 0, attotime::from_msec(1));
	else
		m_sib_timer->reset();

	// Dino derives the sound sample clock from the 9.216 MHz SIB clock.
	// Its 32-bit hold register carries two signed 16-bit mono samples, so
	// the FIFO-full service interrupt occurs at half the sample frequency.
	if (sib_enabled && BIT(m_dino[DINO_SIB_CONTROL], 4))
	{
		unsigned const divisor = BIT(m_dino[DINO_SIB_CONTROL], 8, 7) + 1;
		double const sample_rate = 9'216'000.0 / (32.0 * divisor);
		attotime const interval = attotime::from_hz(sample_rate / 2.0);
		m_dmadac->set_frequency(sample_rate);
		m_microphone_stream->set_sample_rate(sample_rate);
		m_sound_timer->adjust(interval, 0, interval);
	}
	else
	{
		m_sound_timer->reset();
	}

	// The telecom channel is clocked independently of sound, by kSibTelDivMask.
	if (sib_enabled && BIT(m_dino[DINO_SIB_CONTROL], 5))
	{
		unsigned const divisor = BIT(m_dino[DINO_SIB_CONTROL], 16, 7) + 1;
		double const sample_rate = 9'216'000.0 / (32.0 * divisor);
		attotime const interval = attotime::from_hz(sample_rate / 2.0);
		m_telecom_timer->adjust(interval, 0, interval);
	}
	else
	{
		m_telecom_timer->reset();
	}
}


TIMER_CALLBACK_MEMBER(datarover_state::telecom_tick)
{
	if (telecom_dma_running())
	{
		advance_telecom_dma();
		return;
	}

	if (!BIT(m_dino[DINO_SIB_CONTROL], 0) || !BIT(m_dino[DINO_SIB_CONTROL], 5))
		return;

	// Unbuffered telecom: one hold-register slot is free.
	m_dino[DINO_INTERRUPT1] |= INT1_TEL_RECEIVE;
	update_irq();
}


void datarover_state::update_periodic_timer()
{
	u32 const period = m_dino[DINO_PERIODIC_TIMER];

	if (BIT(m_dino[DINO_TIMER_CONTROL], 4) && period)
	{
		attotime const interval = attotime::from_ticks(period, 32'768);
		m_periodic_timer->adjust(interval, 0, interval);
	}
	else
	{
		m_periodic_timer->reset();
	}
}


u64 datarover_state::rtc_ticks() const
{
	static constexpr u64 RTC_MASK = 0x0000'00ff'ffff'ffff;

	if (BIT(m_dino[DINO_TIMER_CONTROL], 3))
		return 0;

	u64 ticks = m_rtc_base;
	if (!BIT(m_dino[DINO_TIMER_CONTROL], 6))
		ticks += u64(machine().time().as_ticks(32'768)) - m_rtc_origin;
	return ticks & RTC_MASK;
}


void datarover_state::persist_rtc()
{
	static constexpr u32 RTC_NVRAM_SIGNATURE = 0x4452'5443; // "DRTC"

	u64 const ticks = rtc_ticks();
	system_time now;
	machine().current_datetime(now);
	u64 const host_time = u64(now.time);

	m_rtc_nvram_data[0] = RTC_NVRAM_SIGNATURE;
	m_rtc_nvram_data[1] = u32(ticks >> 32);
	m_rtc_nvram_data[2] = u32(ticks);
	m_rtc_nvram_data[3] = u32(host_time >> 32);
	m_rtc_nvram_data[4] = u32(host_time);
}


void datarover_state::update_rtc_timers()
{
	static constexpr u64 RTC_MODULUS = 0x0000'0100'0000'0000;

	m_rtc_alarm_timer->reset();
	m_rtc_rollover_timer->reset();
	if (BIT(m_dino[DINO_TIMER_CONTROL], 3) || BIT(m_dino[DINO_TIMER_CONTROL], 6))
		return;

	u64 const current = rtc_ticks();
	u64 const alarm =
			(u64(m_dino[DINO_ALARM_HIGH] & 0xff) << 32)
			| m_dino[DINO_ALARM_LOW];
	u64 const alarm_delta = (alarm - current) & (RTC_MODULUS - 1);
	if (alarm_delta)
		m_rtc_alarm_timer->adjust(attotime::from_ticks(alarm_delta, 32'768));

	u64 const rollover_delta = RTC_MODULUS - current;
	m_rtc_rollover_timer->adjust(attotime::from_ticks(rollover_delta, 32'768));
}


TIMER_CALLBACK_MEMBER(datarover_state::periodic_tick)
{
	m_dino[DINO_INTERRUPT5] |= 0x2000'0000;
	update_irq();
}


TIMER_CALLBACK_MEMBER(datarover_state::rtc_alarm)
{
	m_dino[DINO_INTERRUPT5] |= 0x4000'0000;
	update_irq();
}


TIMER_CALLBACK_MEMBER(datarover_state::rtc_rollover)
{
	m_dino[DINO_INTERRUPT5] |= 0x8000'0000;
	update_rtc_timers();
	update_irq();
}


TIMER_CALLBACK_MEMBER(datarover_state::rtc_persist_tick)
{
	persist_rtc();
}


TIMER_CALLBACK_MEMBER(datarover_state::sib_tick)
{
	for (unsigned slot = 0; slot < m_modem_card.size(); ++slot)
		if (m_modem_card[slot])
			m_modem_card[slot]->poll();

	if (BIT(m_dino[DINO_SIB_CONTROL], 0))
	{
		// Betty's serial bus continually produces subframe boundaries.
		// Schedule them rather than reasserting immediately on clear, allowing
		// the interrupt handler to make forward progress.
		m_dino[DINO_INTERRUPT1] |= 0x0000'0180;
		update_irq();
	}
}


TIMER_CALLBACK_MEMBER(datarover_state::sound_tick)
{
	// Buffered playback owns the sample clock while transmit DMA is enabled;
	// the hold register is only serviced when the OS feeds samples by hand.
	if (sound_dma_running())
	{
		advance_sound_dma();
		return;
	}

	if (!BIT(m_dino[DINO_SIB_CONTROL], 0) || !BIT(m_dino[DINO_SIB_CONTROL], 4))
		return;

	// One 32-bit sound-hold slot (two 16-bit samples) is available.
	m_dino[DINO_INTERRUPT1] |= 0x0000'0400;
	update_irq();
}


void datarover_state::magicbus_set_request(bool asserted)
{
	if (asserted && !magicbus_powered())
		asserted = false;

	if (m_magicbus_request == asserted)
		return;

	m_magicbus_request = asserted;
	m_dino[DINO_INTERRUPT2] |= asserted
			? INT2_MBUS_REQUEST_POS
			: INT2_MBUS_REQUEST_NEG;
	update_irq();
}


TIMER_CALLBACK_MEMBER(datarover_state::magicbus_keyboard_tick)
{
	for (unsigned drained = 0; drained < m_magicbus_key_data.size(); ++drained)
	{
		u8 const data = m_magicbus_keyboard->read();
		if (!data)
			break;

		// The generic AT keyboard powers up with the 0xaa self-test byte, and
		// commands sent by the Magic Bus keyboard client produce 0xfa
		// acknowledgements.  Those are controller protocol bytes, not Set-2
		// key transitions, and the Magic Cap client only accepts the latter.
		if (!magicbus_powered() || data == 0xaa || data == 0xfa)
			continue;

		if (m_magicbus_key_count < m_magicbus_key_data.size())
		{
			m_magicbus_key_data[
					(m_magicbus_key_head + m_magicbus_key_count)
							% m_magicbus_key_data.size()] = data;
			++m_magicbus_key_count;
		}
	}

	if (m_magicbus_key_count && m_magicbus_assigned
			&& BIT(m_magicbus_accessory->read(), 0))
		magicbus_set_request(true);
}


TIMER_CALLBACK_MEMBER(datarover_state::battery_charge_tick)
{
	// The input-port choices are calibrated test levels, not a chemistry
	// simulation.  Four ADC counts per emulated second makes the gradual rise
	// observable in the Power window and deterministic in regressions while
	// preserving the ROM's real 80/320/800 thresholds.
	if (main_battery_charging())
		++m_main_battery_charge;
}


void datarover_state::magicbus_command(u16 command)
{
	if (!magicbus_powered())
	{
		m_magicbus_response = MBUS_RESPONSE_NONE;
		m_magicbus_keyboard_read_pending = false;
		m_magicbus_host_data_pending = false;
		magicbus_set_request(false);
		return;
	}

	// Wire command words are the SDK's command-table entry XORed with the
	// peripheral address code.  This model presents one AT keyboard at
	// address zero; address seven is the broadcast address.
	switch (command)
	{
	case 0xdef0: // command 31, broadcast: assign an address
		// The ROM also starts reinitialization with this broadcast.  Its
		// current client is discarded before the sole attached keyboard is
		// assigned address zero again, so the peripheral must answer even
		// when this machine process has seen an earlier assignment.
		if (BIT(m_magicbus_accessory->read(), 0))
			magicbus_set_request(true);
		break;

	case 0xdca8: // command 24, address 0: take the proposed address
		if (BIT(m_magicbus_accessory->read(), 0) && m_magicbus_request)
		{
			m_magicbus_assigned = true;
			magicbus_set_request(false);
		}
		break;

	case 0xcc5c: // command 12: select the fixed identification record
		if (m_magicbus_assigned)
			m_magicbus_info_page = 1;
		break;

	case 0xcc60: // command 13: select the full peripheral-information record
		if (m_magicbus_assigned)
			m_magicbus_info_page = 2;
		break;

	case 0xcc18: // command 1: read the peripheral request record
		if (m_magicbus_assigned && m_magicbus_request)
		{
			m_magicbus_response = MBUS_RESPONSE_REQUEST;
			m_magicbus_keyboard_read_pending = true;
		}
		break;

	case 0xcc24: // command 2: read the selected data
		if (m_magicbus_assigned)
		{
			if (m_magicbus_keyboard_read_pending)
				m_magicbus_response = MBUS_RESPONSE_KEYBOARD;
			else if (m_magicbus_info_page == 1)
				m_magicbus_response = MBUS_RESPONSE_ID;
			else if (m_magicbus_info_page == 2)
				m_magicbus_response = MBUS_RESPONSE_INFO;
		}
		break;

	case 0xcc3c: // command 5: write an AT-keyboard control packet
		if (m_magicbus_assigned)
			m_magicbus_host_data_pending = true;
		break;

	case 0xdcc8: // command 28, address 0: poll/acknowledge the request line
	case 0xdecc: // command 28, broadcast
		// These delimit the ROM's poll transaction.  The line is cleared
		// when its request record is read and must stay clear while that
		// record waits in the client's software queue.
		break;

	default:
		break;
	}
}


void datarover_state::magicbus_deliver_response()
{
	if (!magicbus_powered())
	{
		m_magicbus_response = MBUS_RESPONSE_NONE;
		return;
	}

	std::array<u8, 88> response{};
	unsigned response_size = 0;

	switch (m_magicbus_response)
	{
	case MBUS_RESPONSE_ID:
		response[0] = 'A';
		response[1] = 'T';
		response[2] = 'K';
		response[3] = 'B';
		response_size = 4;
		break;

	case MBUS_RESPONSE_INFO:
	{
		auto const put_be16 = [&response](unsigned offset, u16 value)
		{
			response[offset] = value >> 8;
			response[offset + 1] = value;
		};
		auto const put_be32 = [&response](unsigned offset, u32 value)
		{
			response[offset] = value >> 24;
			response[offset + 1] = value >> 16;
			response[offset + 2] = value >> 8;
			response[offset + 3] = value;
		};

		// MagicbusPeripheralInfo, reconstructed from the SDK ELF's retained
		// type information.  The three variable strings are empty; the ROM
		// selects its built-in AT-keyboard client from the peripheral ID.
		put_be16(0, 0);
		put_be16(2, 84);
		put_be32(4, 0x4154'4b42); // "ATKB"
		response[8] = 1;          // hardware version
		response[9] = 1;          // software version
		response[10] = 1;         // protocol version
		put_be32(12, 115'200);    // command frequency
		put_be32(16, 10);         // command delay
		put_be32(20, 115'200);    // data-in frequency
		put_be32(24, 10);         // data-in delay
		put_be32(28, 115'200);    // data-out frequency
		put_be32(32, 10);         // data-out delay
		put_be32(36, 10'000);     // request latency
		put_be32(40, 100'000);    // transaction time
		put_be32(44, 10'000);     // active request interval
		put_be32(48, 1'000);      // inactive request interval
		response[64] = 16;        // maximum request record

		u16 checksum = 1;
		for (unsigned offset = 0; offset < 86; ++offset)
			checksum += response[offset];
		put_be16(86, checksum);
		response_size = response.size();
		break;
	}

	case MBUS_RESPONSE_REQUEST:
		// The client dispatches request kind 14 to
		// MagicBusATKeyboard_PeripheralRequest.  Request records are always
		// copied into the ROM's 16-byte queue entry.
		response[1] = 14;
		response_size = 16;
		magicbus_set_request(false);
		break;

	case MBUS_RESPONSE_KEYBOARD:
	{
		unsigned const count = std::min<unsigned>(m_magicbus_key_count, 15);
		response[0] = count;
		for (unsigned index = 0; index < count; ++index)
		{
			response[index + 1] = m_magicbus_key_data[m_magicbus_key_head];
			m_magicbus_key_head =
					(m_magicbus_key_head + 1) % m_magicbus_key_data.size();
		}
		m_magicbus_key_count -= count;
		response_size = 16;
		m_magicbus_keyboard_read_pending = false;
		magicbus_set_request(m_magicbus_key_count != 0);
		break;
	}

	default:
		return;
	}

	m_magicbus_response = MBUS_RESPONSE_NONE;

	if (BIT(m_dino[DINO_MBUS_CONTROL1], 16))
	{
		address_space &space = m_maincpu->space(AS_PROGRAM);
		u32 const start = m_dino[DINO_MBUS_DMA_START] & 0x1fff'fffc;
		unsigned const requested =
				(m_dino[DINO_MBUS_DMA_LENGTH] & 0x000f'fffc) + 4;
		unsigned const count = std::min(response_size, requested);
		for (unsigned offset = 0; offset < count; ++offset)
			space.write_byte(start + offset, response[offset]);

		m_dino[DINO_MBUS_DMA_COUNT] = count >= 4 ? count - 4 : 0;
		m_dino[DINO_INTERRUPT2] |= INT2_MBUS_COMMAND | INT2_MBUS_DMA_END;
	}
	else
	{
		m_dino[DINO_MBUS_DATA] =
				(u32(response[0]) << 24)
				| (u32(response[1]) << 16)
				| (u32(response[2]) << 8)
				| response[3];
		m_dino[DINO_INTERRUPT2] |= INT2_MBUS_COMMAND;
	}
}


void datarover_state::magicbus_accept_host_data()
{
	if (!magicbus_powered()
			|| !m_magicbus_host_data_pending
			|| !BIT(m_dino[DINO_MBUS_CONTROL1], 15))
		return;

	// The ROM's AT-keyboard client wraps controller commands in an eight-byte
	// Magic Bus packet beginning with "K".  MAME already generates Set-2
	// scancodes, so only state-changing commands need forwarding.  Drain the
	// AT controller's acknowledgements through magicbus_keyboard_tick rather
	// than exposing them as keystrokes.
	address_space &space = m_maincpu->space(AS_PROGRAM);
	u32 const start = m_dino[DINO_MBUS_DMA_START] & 0x1fff'fffc;
	unsigned const count =
			(m_dino[DINO_MBUS_DMA_LENGTH] & 0x000f'fffc) + 4;
	if (count >= 3 && space.read_byte(start) == 'K')
	{
		switch (space.read_byte(start + 1))
		{
		case 1: // reset
			m_magicbus_key_head = 0;
			m_magicbus_key_count = 0;
			m_magicbus_keyboard->write(0xff);
			break;

		case 2: // LED state
			m_magicbus_keyboard->write(0xed);
			m_magicbus_keyboard->write(space.read_byte(start + 3) & 7);
			break;

		case 3: // typematic rate, followed by enable
			m_magicbus_keyboard->write(0xf3);
			m_magicbus_keyboard->write(space.read_byte(start + 3));
			if (count >= 5 && space.read_byte(start + 4) == 0xf4)
				m_magicbus_keyboard->write(0xf4);
			break;
		}
	}

	m_magicbus_host_data_pending = false;
	m_dino[DINO_MBUS_DMA_COUNT] = count >= 4 ? count - 4 : 0;
	m_dino[DINO_INTERRUPT2] |= INT2_MBUS_DMA_END;
}


u32 datarover_state::dino_r(offs_t offset, u32 mem_mask)
{
	(void)mem_mask;

	switch (offset)
	{
	case DINO_UART_A_CONTROL1:
		return uart_control_r(0);

	case DINO_UART_B_CONTROL1:
		return uart_control_r(1);

	case DINO_UART_A_HOLD:
		return uart_hold_r(0);

	case DINO_UART_B_HOLD:
		return uart_hold_r(1);

	case DINO_INTERRUPT1:
		return m_dino[DINO_INTERRUPT1];

	case DINO_INTERRUPT2:
		return uart_interrupt_r();

	case DINO_INTERRUPT5:
		return (m_dino[offset] & ~INT5_IR_CARRIER_PIN)
				| (m_irda_carrier->read() ? INT5_IR_CARRIER_PIN : 0);

	case DINO_RTC_HIGH:
		return u32(rtc_ticks() >> 32) & 0xff;

	case DINO_RTC_LOW:
		return u32(rtc_ticks());

	case DINO_MBUS_CONTROL1:
		// kMbusEnabledStatusMask follows the module enable, the transmit FIFO
		// is always empty because commands complete synchronously, and
		// kMbusIntStatusMask is the selected peripheral's request line.
		// TestMBReqLine samples this bit before GetPollingCommand fetches a
		// request record from the attached AT keyboard.
		return (m_dino[offset] & ~DINO_MBUS_STATUS)
				| (BIT(m_dino[offset], 0) ? DINO_MBUS_ENABLED_STATUS : 0)
				| DINO_MBUS_EMPTY_STATUS
				| (m_magicbus_request ? DINO_MBUS_INT_STATUS : 0);

	case DINO_IO_CONTROL:
		// Input bits are sampled independently of the writable GPIO fields.
		// Holding Option during reset enters the IDT monitor; BOOT_MODE keeps
		// that convenient configuration while OPTION_BUTTON models the live
		// physical control used by Magic Cap.  Apollo also wires the two PC
		// Card BVD1 pins here (slot 1 on bit 1, slot 2 on bit 0).
		return (m_dino[offset] & ~(0x0000'000b | DINO_IO_COVER_OPEN))
				| ((m_boot_mode->read() && !m_option_button->read()) ? 0x0000'0008 : 0)
				| (BIT(m_power_supply->read(), 1) ? DINO_IO_COVER_OPEN : 0)
				| (pccard_bvd1_level(0) ? 0x0000'0002 : 0)
				| (pccard_bvd1_level(1) ? 0x0000'0001 : 0);

	case DINO_MFIO_DATA_INPUT:
		// Apollo routes the telephone DAA's ring detector to MFIO pin 0.
		return (m_dino[offset] & ~DINO_MFIO_TELECOM_RING)
				| (m_phone_ring_level ? DINO_MFIO_TELECOM_RING : 0);

	case DINO_POWER_CONTROL:
		// Power-good is a read-only status input.  Without it, the low-level
		// boot path immediately invokes CommonShutdown and restarts forever.
		return (m_dino[offset] & ~(DINO_POWER_ON_BUTTON_STATUS | DINO_POWER_AC_ADAPTER))
				| DINO_POWER_OK_STATUS
				| (m_power_button->read() ? DINO_POWER_ON_BUTTON_STATUS : 0)
				| (BIT(m_power_supply->read(), 0) ? DINO_POWER_AC_ADAPTER : 0);

	default:
		return m_dino[offset];
	}
}


void datarover_state::dino_w(offs_t offset, u32 data, u32 mem_mask)
{
	switch (offset)
	{
	case DINO_UART_A_HOLD:
		uart_hold_w(0, data, mem_mask);
		break;

	case DINO_UART_B_HOLD:
		uart_hold_w(1, data, mem_mask);
		break;

	case DINO_SIB_SF0_AUX:
		COMBINE_DATA(&m_dino[offset]);
		if (ACCESSING_BITS_0_31)
			betty_command(m_dino[offset], false);
		break;

	case DINO_SIB_SF1_AUX:
		COMBINE_DATA(&m_dino[offset]);
		if (ACCESSING_BITS_0_31)
			betty_command(m_dino[offset], true);
		break;

	case DINO_SIB_CONTROL:
	{
		u32 const irq = m_dino[offset] & 0x8000'0000;
		COMBINE_DATA(&m_dino[offset]);
		m_dino[offset] = (m_dino[offset] & ~0x8000'0000) | irq;
		if (BIT(m_dino[offset], 0))
		{
			m_dino[DINO_INTERRUPT1] |= 0x0000'0180;
			if (BIT(m_dino[offset], 4))
				m_dino[DINO_INTERRUPT1] |= 0x0000'0400;
		}
		update_sib_timers();
		break;
	}

	case DINO_SIB_SOUND_HOLD:
		COMBINE_DATA(&m_dino[offset]);
		if (ACCESSING_BITS_0_31 && BIT(m_betty[8], 15))
		{
			std::array<s16, 2> samples{
					s16(m_dino[offset] >> 16),
					s16(m_dino[offset]) };
			m_dmadac->flush();
			m_dmadac->transfer(0, 1, 1, samples.size(), samples.data());
		}
		break;

	case DINO_SIB_DMA:
	{
		// The pointer field is written back by hardware, so a write keeps the
		// current position rather than taking one from the CPU.
		u32 const preserved = m_dino[offset] & (SIB_SOUND_DMA_PTR | SIB_TEL_DMA_PTR);
		bool const was_running = bool(m_dino[offset]
				& (SIB_SOUND_RX_DMA_EN | SIB_SOUND_TX_DMA_EN));
		bool const was_tel_running =
				bool(m_dino[offset] & (SIB_TEL_TX_DMA_EN | SIB_TEL_RX_DMA_EN));
		COMBINE_DATA(&m_dino[offset]);
		m_dino[offset] = (m_dino[offset] & ~(SIB_SOUND_DMA_PTR | SIB_TEL_DMA_PTR))
				| preserved;

		if (!was_running
				&& (m_dino[offset]
						& (SIB_SOUND_RX_DMA_EN | SIB_SOUND_TX_DMA_EN)))
		{
			// A fresh transfer starts at the beginning of the buffer.
			m_dino[offset] &= ~SIB_SOUND_DMA_PTR;
			m_sound_dma_half_signalled = false;
			m_microphone_head = 0;
			m_microphone_count = 0;
			m_microphone_phase = 0;
			m_dmadac->flush();
		}

		if (!was_tel_running
				&& (m_dino[offset] & (SIB_TEL_TX_DMA_EN | SIB_TEL_RX_DMA_EN)))
		{
			m_dino[offset] &= ~SIB_TEL_DMA_PTR;
			m_telecom_dma_half_signalled = false;
			m_telecom_loopback = 0;
		}
		break;
	}

	case DINO_MBUS_CONTROL1:
		// Bits 31:29 are status, so a write must not be able to set them.
		COMBINE_DATA(&m_dino[offset]);
		m_dino[offset] &= ~DINO_MBUS_STATUS;

		// Transfers are synchronous, but staging is observable: the monitor
		// waits for the transmit buffer before writing a command, while the OS
		// waits for empty after that command has entered the shifter.
		if (BIT(m_dino[offset], 0))
		{
			// The IDT monitor waits for "transmit buffer available" before it
			// writes mbusCommand.  Empty belongs to the subsequent command
			// completion, so do not raise it at this staging point.
			m_dino[DINO_INTERRUPT2] |= INT2_MBUS_TRANSMIT;
			if (m_magicbus_response != MBUS_RESPONSE_NONE)
				magicbus_deliver_response();
			magicbus_accept_host_data();
		}
		break;

	case DINO_MBUS_COMMAND:
		COMBINE_DATA(&m_dino[offset]);
		if (ACCESSING_BITS_0_15)
		{
			u16 const command = m_dino[offset];
			m_dino[DINO_INTERRUPT2] |= INT2_MBUS_TRANSMIT | INT2_MBUS_EMPTY;
			magicbus_command(command);
		}
		break;

	case DINO_MBUS_DATA:
		COMBINE_DATA(&m_dino[offset]);
		if (m_magicbus_host_data_pending)
		{
			// Type-2 writes of four bytes or fewer use the hold register
			// rather than DMA.  No current ATKB control packet is that short,
			// but completing it here preserves the controller semantics.
			m_magicbus_host_data_pending = false;
			m_dino[DINO_INTERRUPT2] |= INT2_MBUS_EMPTY;
		}
		break;

	case DINO_TIMER_CONTROL:
	{
		u32 const old_control = m_dino[offset];
		u64 const current_ticks = rtc_ticks();
		COMBINE_DATA(&m_dino[offset]);
		if (BIT(m_dino[offset], 3))
		{
			m_rtc_base = 0;
			m_rtc_origin = machine().time().as_ticks(32'768);
		}
		else if (BIT(old_control, 3) || (BIT(old_control, 6) != BIT(m_dino[offset], 6)))
		{
			m_rtc_base = BIT(old_control, 3) ? 0 : current_ticks;
			m_rtc_origin = machine().time().as_ticks(32'768);
		}
		update_periodic_timer();
		update_rtc_timers();
		break;
	}

	case DINO_PERIODIC_TIMER:
		COMBINE_DATA(&m_dino[offset]);
		update_periodic_timer();
		break;

	case DINO_MFIO_DATA_OUTPUT:
	{
		u32 const old_output = m_dino[offset];
		COMBINE_DATA(&m_dino[offset]);
		if (!(old_output & DINO_MFIO_MBUS_VCC_OFF)
				&& (m_dino[offset] & DINO_MFIO_MBUS_VCC_OFF))
		{
			// Removing accessory power loses its address and pending
			// transaction.  A later broadcast FIND discovers it afresh.
			m_magicbus_assigned = false;
			m_magicbus_info_page = 0;
			m_magicbus_response = MBUS_RESPONSE_NONE;
			m_magicbus_keyboard_read_pending = false;
			m_magicbus_host_data_pending = false;
			m_magicbus_key_head = 0;
			m_magicbus_key_count = 0;
			magicbus_set_request(false);
		}
		break;
	}

	case DINO_ALARM_HIGH:
	case DINO_ALARM_LOW:
		COMBINE_DATA(&m_dino[offset]);
		update_rtc_timers();
		break;

	case DINO_POWER_CONTROL:
	{
		u32 const old_control = m_dino[offset];
		u32 const write_mask = mem_mask & DINO_POWER_WRITE_MASK;
		m_dino[offset] = (old_control & ~write_mask) | (data & write_mask);

		// The low-level Betty reset uses Dino's stop timer as a short,
		// polled delay.  Complete it synchronously until the stop timer gets
		// its own scheduled counter.
		if (BIT(m_dino[offset], 11))
			m_dino[DINO_INTERRUPT5] |= 0x1000'0000;
		if (BIT(m_dino[offset], 10) && BIT(m_dino[offset], 9))
		{
			// Force shutdown follows Magic Cap's heap/vault preparation.
			// Once Dino acknowledges it, the physical machine loses VCC.
			m_dino[offset] |= 0x0000'0100;
			persist_rtc();
			machine().schedule_exit();
		}
		else if ((!BIT(old_control, 4) && BIT(m_dino[offset], 4))
				|| (BIT(old_control, 0) && !BIT(m_dino[offset], 0)))
		{
			// StopCpu is Doze's stop request.  Removing VCC also stops the
			// CPU even if a pending interrupt released an earlier StopCpu.
			persist_rtc();
			m_maincpu->suspend(SUSPEND_REASON_HALT, true);
		}
		break;
	}

	case DINO_INTERRUPT1:
		m_dino[offset] &= ~(data & mem_mask);
		break;

	case DINO_INTERRUPT2:
	case DINO_INTERRUPT3:
	case DINO_INTERRUPT4:
	case DINO_INTERRUPT5:
		// Dino interrupt status registers are write-to-clear.
		m_dino[offset] &= ~(data & mem_mask);
		break;

	case DINO_INTERRUPT6:
		// Bank 6 is the read-only priority summary.
		break;

	default:
		COMBINE_DATA(&m_dino[offset]);
		break;
	}

	update_irq();
}


u32 datarover_state::glacier_r(unsigned slot, offs_t offset) const
{
	return (slot ? m_glacier2 : m_glacier1)[offset];
}


void datarover_state::glacier_w(
		unsigned slot,
		offs_t offset,
		u32 data,
		u32 mem_mask)
{
	std::array<u32, 0x24 / 4> &glacier = slot ? m_glacier2 : m_glacier1;
	if (offset == GLACIER_IO_DATA_INPUT)
		return;

	if (offset == GLACIER_IO_POS_STATUS || offset == GLACIER_IO_NEG_STATUS)
		glacier[offset] &= ~(data & mem_mask);
	else
		COMBINE_DATA(&glacier[offset]);

	update_glacier_irq();
	update_irq();
}


void datarover_state::update_glacier_irq()
{
	bool pending = false;
	for (std::array<u32, 0x24 / 4> const *glacier :
			{ &m_glacier1, &m_glacier2 })
	{
		if (BIT((*glacier)[GLACIER_CONTROL] >> 16, 1))
		{
			pending |= bool(
					((*glacier)[GLACIER_IO_POS_ENABLE]
							& (*glacier)[GLACIER_IO_POS_STATUS])
					| ((*glacier)[GLACIER_IO_NEG_ENABLE]
							& (*glacier)[GLACIER_IO_NEG_STATUS]));
		}
	}

	if (pending)
		m_dino[DINO_INTERRUPT3] |= 0x0000'0004;
	else
		m_dino[DINO_INTERRUPT3] &= ~0x0000'0004;
}


u32 datarover_state::glacier1_r(offs_t offset, u32 mem_mask)
{
	(void)mem_mask;
	return glacier_r(0, offset);
}


void datarover_state::glacier1_w(offs_t offset, u32 data, u32 mem_mask)
{
	glacier_w(0, offset, data, mem_mask);
}


u32 datarover_state::glacier2_r(offs_t offset, u32 mem_mask)
{
	(void)mem_mask;
	return glacier_r(1, offset);
}


void datarover_state::glacier2_w(offs_t offset, u32 data, u32 mem_mask)
{
	glacier_w(1, offset, data, mem_mask);
}


void datarover_state::machine_start()
{
	for (device_pty_interface &pty : pty_interface_enumerator(machine().root_device()))
		osd_printf_info("%s PTY: %s\n", pty.device().tag(), pty.slave_name());

	// The 840F places four byte-wide 16-Mbit flash devices on Dino's 32-bit
	// ROM bus.  Build each device's first-run NVRAM image from its byte lane;
	// subsequent runs load the independently writable flash NVRAM instead.
	if (m_flash[0])
	{
		u8 const *const source = memregion("maincpu")->base();
		for (unsigned lane = 0; lane < m_flash.size(); ++lane)
		{
			u8 *const target = memregion(m_flash[lane]->tag())->base();
			for (u32 offset = 0; offset < 0x20'0000; ++offset)
				target[offset] = source[(offset * 4) + lane];
		}
	}

	m_periodic_timer = timer_alloc(FUNC(datarover_state::periodic_tick), this);
	m_rtc_alarm_timer = timer_alloc(FUNC(datarover_state::rtc_alarm), this);
	m_rtc_rollover_timer = timer_alloc(FUNC(datarover_state::rtc_rollover), this);
	m_rtc_persist_timer = timer_alloc(FUNC(datarover_state::rtc_persist_tick), this);
	m_sib_timer = timer_alloc(FUNC(datarover_state::sib_tick), this);
	m_sound_timer = timer_alloc(FUNC(datarover_state::sound_tick), this);
	m_telecom_timer = timer_alloc(FUNC(datarover_state::telecom_tick), this);
	m_magicbus_keyboard_timer =
			timer_alloc(FUNC(datarover_state::magicbus_keyboard_tick), this);
	m_battery_charge_timer =
			timer_alloc(FUNC(datarover_state::battery_charge_tick), this);
	m_rtc_nvram->set_base(m_rtc_nvram_data.data(), sizeof(m_rtc_nvram_data));
	m_dmadac->set_frequency(11'025);
	m_dmadac->enable(1);
	// Keep one un-routed output so MAME can safely resample this input stream
	// when Magic Cap selects a different recording rate.
	m_microphone_stream = stream_alloc(1, 1, 11'025);

	save_item(NAME(m_dino));
	save_item(NAME(m_glacier1));
	save_item(NAME(m_glacier2));
	save_item(NAME(m_betty));
	save_item(NAME(m_betty_pos_pending));
	save_item(NAME(m_betty_neg_pending));
	save_item(NAME(m_phone_ring_level));
	save_item(NAME(m_uart_rx_data));
	save_item(NAME(m_uart_rx_head));
	save_item(NAME(m_uart_rx_count));
	save_item(NAME(m_pccard_cd1));
	save_item(NAME(m_pccard_cd2));
	save_item(NAME(m_pccard_ready));
	save_item(NAME(m_pccard_bvd1));
	save_item(NAME(m_pccard_bvd2));
	save_item(NAME(m_pccard_wp));
	save_item(NAME(m_rtc_nvram_data));
	save_item(NAME(m_rtc_base));
	save_item(NAME(m_rtc_origin));
	save_item(NAME(m_sound_dma_half_signalled));
	save_item(NAME(m_microphone_data));
	save_item(NAME(m_microphone_head));
	save_item(NAME(m_microphone_count));
	save_item(NAME(m_microphone_phase));
	save_item(NAME(m_telecom_dma_half_signalled));
	save_item(NAME(m_telecom_loopback));
	save_item(NAME(m_magicbus_request));
	save_item(NAME(m_magicbus_assigned));
	save_item(NAME(m_magicbus_info_page));
	save_item(NAME(m_magicbus_response));
	save_item(NAME(m_magicbus_keyboard_read_pending));
	save_item(NAME(m_magicbus_host_data_pending));
	save_item(NAME(m_magicbus_key_data));
	save_item(NAME(m_magicbus_key_head));
	save_item(NAME(m_magicbus_key_count));
	save_item(NAME(m_main_battery_charge));
	machine().save().register_postload(
			save_prepost_delegate(FUNC(datarover_state::restore_inputs), this));
}


void datarover_state::machine_reset()
{
	bool retained_ram = false;
	for (unsigned offset = 0; offset < (0x0040'0000 / 4); ++offset)
	{
		if (m_ram[offset])
		{
			retained_ram = true;
			break;
		}
	}

	m_dino.fill(0);
	// Both transmit holding registers are empty after reset.  Magic Cap's
	// serial-server initialization explicitly clears these status bits before
	// enabling an interrupt-driven transfer; the IDT monitor polls them.
	m_dino[DINO_INTERRUPT2] = 0x0501'4000;
	m_glacier1.fill(0);
	m_glacier2.fill(0);
	m_glacier1[GLACIER_IO_DATA_INPUT] = 0x0c00'0000;
	m_glacier2[GLACIER_IO_DATA_INPUT] = 0x0c00'0000;
	m_betty.fill(0);
	m_betty_pos_pending = 0;
	m_betty_neg_pending = 0;
	m_phone_ring_level = bool(m_phone_ring->read());
	for (std::array<u8, UART_RX_QUEUE_SIZE> &data : m_uart_rx_data)
		data.fill(0);
	m_uart_rx_head.fill(0);
	m_uart_rx_count.fill(0);
	m_periodic_timer->reset();
	m_rtc_alarm_timer->reset();
	m_rtc_rollover_timer->reset();
	m_rtc_persist_timer->adjust(attotime::from_seconds(1), 0, attotime::from_seconds(1));
	m_sib_timer->reset();
	m_sound_timer->reset();
	m_sound_dma_half_signalled = false;
	m_microphone_data.fill(0);
	m_microphone_head = 0;
	m_microphone_count = 0;
	m_microphone_phase = 0;
	m_telecom_timer->reset();
	m_telecom_dma_half_signalled = false;
	m_telecom_loopback = 0;
	m_magicbus_request = false;
	m_magicbus_assigned = false;
	m_magicbus_info_page = 0;
	m_magicbus_response = MBUS_RESPONSE_NONE;
	m_magicbus_keyboard_read_pending = false;
	m_magicbus_host_data_pending = false;
	m_magicbus_key_head = 0;
	m_magicbus_key_count = 0;
	m_magicbus_keyboard_timer->adjust(
			attotime::from_msec(1), 0, attotime::from_hz(240));
	m_battery_charge_timer->adjust(
			attotime::from_msec(250), 0, attotime::from_msec(250));
	m_dmadac->enable(0);
	m_dmadac->set_frequency(11'025);
	m_dmadac->enable(1);

	static constexpr u32 RTC_NVRAM_SIGNATURE = 0x4452'5443; // "DRTC"
	static constexpr u64 RTC_MASK = 0x0000'00ff'ffff'ffff;
	m_rtc_base = 0;
	if (m_rtc_nvram_data[0] == RTC_NVRAM_SIGNATURE)
	{
		u64 const saved_ticks =
				(u64(m_rtc_nvram_data[1]) << 32) | m_rtc_nvram_data[2];
		u64 const saved_host_time =
				(u64(m_rtc_nvram_data[3]) << 32) | m_rtc_nvram_data[4];
		s64 elapsed = 0;
		if (BIT(m_rtc_resume->read(), 0))
		{
			system_time now;
			machine().current_datetime(now);
			elapsed = now.time - s64(saved_host_time);
		}
		m_rtc_base = (saved_ticks + (u64(std::max<s64>(elapsed, 0)) * 32'768)) & RTC_MASK;
	}
	m_rtc_origin = machine().time().as_ticks(32'768);

	// `touch_init` accepts revision 0x1002 or 0x1003.
	m_betty[12] = 0x1002;
	set_phone_line(bool(m_phone_line->read()), false);

	// Main DRAM is battery-backed.  A zero-filled new NVRAM image is a cold
	// start; loaded state retains the heap and follows Dino's warm-start path.
	m_dino[DINO_POWER_CONTROL] = retained_ram ? 0x2000'0001 : 0x2000'0005;

	// PC Card CD1/CD2 are active low. Glacier exposes its 16-bit registers on
	// the upper half of each 32-bit word in the CPU map.
	update_pccard_inputs(0);
	update_pccard_inputs(1);
	update_rtc_timers();
	update_irq();
}


static INPUT_PORTS_START(datarover840)
	PORT_START("BOOT_MODE")
	PORT_CONFNAME(0x08, 0x08, "Power-on mode")
	PORT_CONFSETTING(0x08, "Magic Cap")
	PORT_CONFSETTING(0x00, "IDT monitor")

	PORT_START("OPTION_BUTTON")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Option button") PORT_CODE(KEYCODE_LALT) PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(datarover_state::option_changed), 0)

	PORT_START("PCCARD1_BATTERY")
	PORT_CONFNAME(0x03, 0x03, "PC Card slot 1 battery")
	PORT_CONFSETTING(0x03, "Good")
	PORT_CONFSETTING(0x01, "Low")
	PORT_CONFSETTING(0x02, "Dead")
	PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(datarover_state::pccard_battery_changed), 0)

	PORT_START("PCCARD2_BATTERY")
	PORT_CONFNAME(0x03, 0x03, "PC Card slot 2 battery")
	PORT_CONFSETTING(0x03, "Good")
	PORT_CONFSETTING(0x01, "Low")
	PORT_CONFSETTING(0x02, "Dead")
	PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(datarover_state::pccard_battery_changed), 1)

	PORT_START("TOUCH_X")
	PORT_BIT(0xffff, 0x8000, IPT_LIGHTGUN_X) PORT_NAME("Pen X") PORT_MINMAX(0, 0xffff) PORT_SENSITIVITY(100) PORT_CODE(GUNCODE_X) PORT_CODE_DEC(INPUT_CODE_INVALID) PORT_CODE_INC(INPUT_CODE_INVALID) PORT_CROSSHAIR(X, 1.0, 0.0, 0)

	PORT_START("TOUCH_Y")
	PORT_BIT(0xffff, 0x8000, IPT_LIGHTGUN_Y) PORT_NAME("Pen Y") PORT_MINMAX(0, 0xffff) PORT_SENSITIVITY(100) PORT_CODE(GUNCODE_Y) PORT_CODE_DEC(INPUT_CODE_INVALID) PORT_CODE_INC(INPUT_CODE_INVALID) PORT_CROSSHAIR(Y, 1.0, 0.0, 0)

	PORT_START("TOUCH_BUTTON")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_BUTTON1) PORT_NAME("Touch screen") PORT_CODE(GUNCODE_BUTTON1) PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(datarover_state::touch_changed), 0)

	PORT_START("POWER_BUTTON")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Power button") PORT_CODE(KEYCODE_END) PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(datarover_state::power_changed), 0)

	PORT_START("PHONE_LINE")
	PORT_CONFNAME(0x01, 0x01, "Phone line")
	PORT_CONFSETTING(0x01, "Connected")
	PORT_CONFSETTING(0x00, "Disconnected")
	PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(datarover_state::phone_line_changed), 0)

	PORT_START("PHONE_RING")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Incoming telephone ring") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(datarover_state::phone_ring_changed), 0)

	// Resuming battery-backed state normally advances the RTC by the host
	// wall-clock time that passed while the machine was off, which is what a
	// real communicator does.  That makes every run of a headless regression
	// see a different time of day, so automated checks can pin the clock to the
	// value stored in NVRAM instead and get reproducible behavior.
	// Battery levels are what the OS samples on Betty ADC inputs 24 and 28.
	// The defaults are healthy; the other settings exist so the low-battery
	// paths can be exercised without waiting for a cell to run down.
	PORT_START("BATTERY")
	PORT_CONFNAME(0x03, 0x00, "Main battery")
	PORT_CONFSETTING(0x00, "Full")
	PORT_CONFSETTING(0x01, "Low")
	PORT_CONFSETTING(0x02, "Empty")
	PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(datarover_state::main_battery_changed), 0)
	PORT_CONFNAME(0x0c, 0x00, "Backup battery")
	PORT_CONFSETTING(0x00, "Good")
	PORT_CONFSETTING(0x04, "Low")
	PORT_CONFSETTING(0x08, "Empty")

	// External power and the battery cover are inputs PowerSupplyGen2MFS
	// reads.  The defaults describe a communicator running on its own cells
	// with the cover fitted, which is what the boot path expects.
	PORT_START("POWER_SUPPLY")
	PORT_CONFNAME(0x01, 0x00, "AC adapter")
	PORT_CONFSETTING(0x00, "Detached")
	PORT_CONFSETTING(0x01, "Attached")
	PORT_CONFNAME(0x02, 0x00, "Battery cover")
	PORT_CONFSETTING(0x00, "Fitted")
	PORT_CONFSETTING(0x02, "Removed")
	PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(datarover_state::battery_cover_changed), 0)

	PORT_START("RTC_RESUME")
	PORT_CONFNAME(0x01, 0x01, "RTC on resume")
	PORT_CONFSETTING(0x01, "Advance by host clock")
	PORT_CONFSETTING(0x00, "Freeze at saved value")

	PORT_START("MAGICBUS_ACCESSORY")
	PORT_CONFNAME(0x01, 0x01, "Magic Bus accessory")
	PORT_CONFSETTING(0x01, "AT keyboard")
	PORT_CONFSETTING(0x00, "None")

	PORT_START("IRDA_CARRIER")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_OTHER)
		PORT_NAME("IrDA carrier")
		PORT_CHANGED_MEMBER(
				DEVICE_SELF, FUNC(datarover_state::irda_carrier_changed), 0)

	PORT_START("MICROPHONE_SOURCE")
	PORT_CONFNAME(0x03, 0x00, "Microphone source")
	PORT_CONFSETTING(0x00, "Host microphone")
	PORT_CONFSETTING(0x01, "1 kHz test tone")
	PORT_CONFSETTING(0x02, "Silence")
INPUT_PORTS_END


static DEVICE_INPUT_DEFAULTS_START(datarover_rs232_defaults)
	DEVICE_INPUT_DEFAULTS("RS232_TXBAUD", 0xff, RS232_BAUD_19200)
	DEVICE_INPUT_DEFAULTS("RS232_RXBAUD", 0xff, RS232_BAUD_19200)
	DEVICE_INPUT_DEFAULTS("RS232_DATABITS", 0xff, RS232_DATABITS_8)
	DEVICE_INPUT_DEFAULTS("RS232_PARITY", 0xff, RS232_PARITY_NONE)
	DEVICE_INPUT_DEFAULTS("RS232_STOPBITS", 0xff, RS232_STOPBITS_1)
DEVICE_INPUT_DEFAULTS_END


static void datarover_pccards(device_slot_interface &device)
{
	device.option_add("3c589", ETHERLINK_III_PCCARD);
	device.option_add("linear", DATAROVER_LINEAR_PCCARD);
	device.option_add("modem", DATAROVER_MODEM_PCCARD);
}


void datarover_state::datarover840(machine_config &config)
{
	// Keep the handheld LCD as the initial view.  The generic terminal is a
	// useful monitor/debugging surface, but normal Magic Cap boot leaves it
	// blank and its black raster is easily mistaken for a failed LCD.
	config.set_default_layout(layout_datarover840);

	R3900(config, m_maincpu, 36.864_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &datarover_state::memory_map);

	// The communicator's main DRAM is battery-backed; preserving it retains
	// the Magic Cap heap, calibration and user state across power cycles.
	NVRAM(config, "ram", nvram_device::DEFAULT_ALL_0);
	NVRAM(config, m_rtc_nvram, nvram_device::DEFAULT_ALL_0);

	PCCARD_SLOT(config, m_pccard[0], datarover_pccards, nullptr);
	m_pccard[0]->cd1().set(FUNC(datarover_state::pccard_cd1_w<0>));
	m_pccard[0]->cd2().set(FUNC(datarover_state::pccard_cd2_w<0>));
	m_pccard[0]->bvd1().set(FUNC(datarover_state::pccard_bvd1_w<0>));
	m_pccard[0]->bvd2().set(FUNC(datarover_state::pccard_bvd2_w<0>));
	m_pccard[0]->wp().set(FUNC(datarover_state::pccard_wp_w<0>));

	PCCARD_SLOT(config, m_pccard[1], datarover_pccards, nullptr);
	m_pccard[1]->cd1().set(FUNC(datarover_state::pccard_cd1_w<1>));
	m_pccard[1]->cd2().set(FUNC(datarover_state::pccard_cd2_w<1>));
	m_pccard[1]->bvd1().set(FUNC(datarover_state::pccard_bvd1_w<1>));
	m_pccard[1]->bvd2().set(FUNC(datarover_state::pccard_bvd2_w<1>));
	m_pccard[1]->wp().set(FUNC(datarover_state::pccard_wp_w<1>));

	SCREEN(config, m_screen, SCREEN_TYPE_LCD);
	m_screen->set_refresh_hz(60);
	m_screen->set_size(480, 320);
	m_screen->set_visarea_full();
	m_screen->set_screen_update(FUNC(datarover_state::screen_update));

	GENERIC_TERMINAL(config, m_terminal);
	m_terminal->set_keyboard_callback(FUNC(datarover_state::terminal_key));

	AT_KEYB(config, m_magicbus_keyboard, pc_keyboard_device::KEYBOARD_TYPE::AT, 2);

	for (unsigned channel = 0; channel < 2; ++channel)
	{
		DATAROVER_UART(config, m_uart[channel], 0);
		RS232_PORT(config, m_rs232[channel], default_rs232_devices, nullptr);
		m_rs232[channel]->set_option_device_input_defaults(
				"null_modem",
				DEVICE_INPUT_DEFAULTS_NAME(datarover_rs232_defaults));
		m_rs232[channel]->set_option_device_input_defaults(
				"pty",
				DEVICE_INPUT_DEFAULTS_NAME(datarover_rs232_defaults));
		m_uart[channel]->txd_handler().set(
				m_rs232[channel],
				FUNC(rs232_port_device::write_txd));
		m_rs232[channel]->rxd_handler().set(
				m_uart[channel],
				FUNC(datarover_uart_device::input_txd));
	}
	m_uart[0]->received_handler().set(FUNC(datarover_state::uart_received<0>));
	m_uart[1]->received_handler().set(FUNC(datarover_state::uart_received<1>));
	DATAROVER_IRDA(config, m_irda, 0);
	m_irda->received_handler().set(FUNC(datarover_state::irda_received));

	SPEAKER(config, "speaker").front_center();
	DMADAC(config, m_dmadac).add_route(ALL_OUTPUTS, "speaker", 0.5);
	MICROPHONE(config, m_microphone, 1).front_center();
	m_microphone->add_route(0, *this, 1.0, 0);
}


void datarover_state::datarover840f(machine_config &config)
{
	datarover840(config);
	m_maincpu->set_addrmap(AS_PROGRAM, &datarover_state::flash_memory_map);
	for (auto &flash : m_flash)
		FUJITSU_29F016A(config, flash);
}


ROM_START(datarover840)
	ROM_REGION32_BE(0x800000, "maincpu", ROMREGION_ERASEFF)
	ROM_LOAD("magiccap-usa.image", 0x000000, 0x451817, CRC(0f9192ed) SHA1(4fc33eb40db75a0930861df383b109e46ae4ad91))
ROM_END


ROM_START(datarover840f)
	ROM_REGION32_BE(0x800000, "maincpu", ROMREGION_ERASEFF)
	ROM_LOAD("magiccap-usa.image", 0x000000, 0x451817, CRC(0f9192ed) SHA1(4fc33eb40db75a0930861df383b109e46ae4ad91))
	ROM_REGION(0x200000, "flash0", ROMREGION_ERASEFF)
	ROM_REGION(0x200000, "flash1", ROMREGION_ERASEFF)
	ROM_REGION(0x200000, "flash2", ROMREGION_ERASEFF)
	ROM_REGION(0x200000, "flash3", ROMREGION_ERASEFF)
ROM_END


ROM_START(datarover840j)
	ROM_REGION32_BE(0x800000, "maincpu", ROMREGION_ERASEFF)
	ROM_LOAD("magiccap-japan.image", 0x000000, 0x5cf318, CRC(ba5df7d1) SHA1(fbf7161681c799a33f896df4910c329ad2e0453c))
ROM_END


// Development build dated 1998-04-07, from the Apollo (DataRover) debugger
// directory of the Mac Rosemary SDK. Same base address and BootCap entry as
// the release image, but 357 KiB larger: it retains the OS test framework
// (28 test suites, unit tests, heap inspector, input journaling) that the
// shipping ROM omits. Useful as a source of ROM-provided self-tests.
ROM_START(datarover840d)
	ROM_REGION32_BE(0x800000, "maincpu", ROMREGION_ERASEFF)
	ROM_LOAD("magiccap-usa-dev.image", 0x000000, 0x4a8ad7, CRC(98282a67) SHA1(443ab11e296c4ab5b361be4265e3fd2f72212f11))
ROM_END

} // anonymous namespace


//    YEAR  NAME           PARENT        COMPAT  MACHINE        INPUT         CLASS             INIT        COMPANY          FULLNAME                       FLAGS
COMP( 1998, datarover840,  0,            0,      datarover840,  datarover840, datarover_state, empty_init, "General Magic", "DataRover 840",               MACHINE_NOT_WORKING | MACHINE_SUPPORTS_SAVE )
COMP( 1998, datarover840f, datarover840, 0,      datarover840f, datarover840, datarover_state, empty_init, "General Magic", "DataRover 840F (flash)",       MACHINE_NOT_WORKING | MACHINE_SUPPORTS_SAVE )
COMP( 1998, datarover840j, datarover840, 0,      datarover840,  datarover840, datarover_state, empty_init, "General Magic", "DataRover 840 (Japan ROM)",    MACHINE_NOT_WORKING | MACHINE_SUPPORTS_SAVE )
COMP( 1998, datarover840d, datarover840, 0,      datarover840,  datarover840, datarover_state, empty_init, "General Magic", "DataRover 840 (development ROM, 1998-04-07)", MACHINE_NOT_WORKING | MACHINE_SUPPORTS_SAVE )
