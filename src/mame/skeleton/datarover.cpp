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
      - connect touchscreen, RTC, sound, modem and PC Card slots

***************************************************************************/

#include "emu.h"

#include "cpu/mips/mips1.h"
#include "machine/terminal.h"

#include "screen.h"

#include "datarover840.lh"

#include <array>


namespace {

class datarover_state : public driver_device
{
public:
	datarover_state(machine_config const &mconfig, device_type type, char const *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_screen(*this, "screen")
		, m_terminal(*this, "terminal")
		, m_ram(*this, "ram")
		, m_rom(*this, "maincpu")
		, m_boot_mode(*this, "BOOT_MODE")
	{
	}

	void datarover840(machine_config &config);

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	static constexpr u32 DINO_UART_A_CONTROL1 = 0x0b0 / 4;
	static constexpr u32 DINO_UART_A_HOLD = 0x0c4 / 4;
	static constexpr u32 DINO_UART_B_CONTROL1 = 0x0c8 / 4;
	static constexpr u32 DINO_UART_B_HOLD = 0x0dc / 4;
	static constexpr u32 DINO_SIB_SF0_AUX = 0x080 / 4;
	static constexpr u32 DINO_SIB_SF1_AUX = 0x084 / 4;
	static constexpr u32 DINO_SIB_SF0_STATUS = 0x088 / 4;
	static constexpr u32 DINO_SIB_SF1_STATUS = 0x08c / 4;
	static constexpr u32 DINO_SIB_CONTROL = 0x074 / 4;
	static constexpr u32 DINO_VIDEO_HIGH_BUFFER = 0x030 / 4;
	static constexpr u32 DINO_MBUS_CONTROL1 = 0x0e0 / 4;
	static constexpr u32 DINO_INTERRUPT1 = 0x100 / 4;
	static constexpr u32 DINO_INTERRUPT2 = 0x104 / 4;
	static constexpr u32 DINO_INTERRUPT3 = 0x108 / 4;
	static constexpr u32 DINO_INTERRUPT4 = 0x10c / 4;
	static constexpr u32 DINO_INTERRUPT5 = 0x110 / 4;
	static constexpr u32 DINO_INTERRUPT6 = 0x114 / 4;
	static constexpr u32 DINO_RTC_HIGH = 0x140 / 4;
	static constexpr u32 DINO_RTC_LOW = 0x144 / 4;
	static constexpr u32 DINO_TIMER_CONTROL = 0x150 / 4;
	static constexpr u32 DINO_IO_CONTROL = 0x180 / 4;
	static constexpr u32 DINO_POWER_CONTROL = 0x1c4 / 4;
	static constexpr u32 GLACIER_IO_DATA_INPUT = 0x00c / 4;
	static constexpr u32 VECTOR_PAGE_ROM_OFFSET = 0x0029'6400;
	static constexpr u32 VECTOR_PAGE_RAM_OFFSET = 0x0000'0200;

	void memory_map(address_map &map) ATTR_COLD;

	u32 dino_r(offs_t offset, u32 mem_mask = ~0U);
	void dino_w(offs_t offset, u32 data, u32 mem_mask = ~0U);
	u32 glacier1_r(offs_t offset, u32 mem_mask = ~0U);
	void glacier1_w(offs_t offset, u32 data, u32 mem_mask = ~0U);
	u32 glacier2_r(offs_t offset, u32 mem_mask = ~0U);
	void glacier2_w(offs_t offset, u32 data, u32 mem_mask = ~0U);
	u32 vector_page_r(offs_t offset, u32 mem_mask = ~0U);
	void vector_page_w(offs_t offset, u32 data, u32 mem_mask = ~0U);

	void betty_command(u32 command, bool subframe1);
	u32 uart_interrupt_r() const;
	u32 uart_control_r(unsigned channel) const;
	u32 uart_hold_r(unsigned channel);
	void uart_hold_w(unsigned channel, u32 data, u32 mem_mask);
	void terminal_key(u8 data);
	u32 screen_update(screen_device &screen, bitmap_rgb32 &bitmap, rectangle const &cliprect);

	required_device<r3900_device> m_maincpu;
	required_device<screen_device> m_screen;
	required_device<generic_terminal_device> m_terminal;
	required_shared_ptr<u32> m_ram;
	required_region_ptr<u32> m_rom;
	required_ioport m_boot_mode;

	std::array<u32, 0x200 / 4> m_dino{};
	std::array<u32, 0x24 / 4> m_glacier1{};
	std::array<u32, 0x24 / 4> m_glacier2{};
	std::array<u16, 16> m_betty{};
	std::array<u8, 2> m_uart_rx_data{};
	std::array<bool, 2> m_uart_rx_ready{};
	u64 m_rtc_origin = 0;
};


void datarover_state::memory_map(address_map &map)
{
	map.unmap_value_high();

	map(0x00000000, 0x003fffff).ram().share("ram");

	map(0x10400000, 0x10400023).rw(
			FUNC(datarover_state::glacier1_r),
			FUNC(datarover_state::glacier1_w));
	map(0x10800000, 0x10800023).rw(
			FUNC(datarover_state::glacier2_r),
			FUNC(datarover_state::glacier2_w));
	map(0x10c00000, 0x10c001ff).rw(
			FUNC(datarover_state::dino_r),
			FUNC(datarover_state::dino_w));

	map(0x13c00000, 0x13e963ff).rom().region("maincpu", 0);
	map(0x13e96400, 0x13e965ff).rw(
			FUNC(datarover_state::vector_page_r),
			FUNC(datarover_state::vector_page_w));
	map(0x13e96600, 0x143fffff).rom().region("maincpu", 0x00296600);

	// Architectural reset vector alias.  Only the first word and delay slot
	// are required before the ROM jumps to its normal 0xb3c00000 alias.
	map(0x1fc00000, 0x1fc00007).rom().region("maincpu", 0);
}


u32 datarover_state::vector_page_r(offs_t offset, u32 mem_mask)
{
	(void)mem_mask;

	if (BIT(m_dino[0], 25))
		return m_ram[(VECTOR_PAGE_RAM_OFFSET / 4) + offset];

	return m_rom[(VECTOR_PAGE_ROM_OFFSET / 4) + offset];
}


void datarover_state::vector_page_w(offs_t offset, u32 data, u32 mem_mask)
{
	// SetupVectorDispatching copies this ROM page to RAM at 0x200, then
	// enables Dino address-remap region 1 before patching its dispatch stub.
	if (BIT(m_dino[0], 25))
		COMBINE_DATA(&m_ram[(VECTOR_PAGE_RAM_OFFSET / 4) + offset]);
}


u32 datarover_state::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, rectangle const &cliprect)
{
	static constexpr std::array<u8, 4> LEVEL{ 0xff, 0xaa, 0x55, 0x00 };
	static constexpr u32 FALLBACK_BASE = 0x003f'6a00;
	static constexpr u32 FRAME_BYTES = 480 * 320 / 4;

	(void)screen;

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
	// Report transmitter ready/empty using both status encodings found in
	// the monitor's Sony/Toshiba tables.
	u32 result = 0x05014000;
	if (m_uart_rx_ready[channel])
		result |= 0x80200000;

	return result;
}


u32 datarover_state::uart_hold_r(unsigned channel)
{
	u8 const data = m_uart_rx_data[channel];
	m_uart_rx_ready[channel] = false;
	return data;
}


void datarover_state::uart_hold_w(unsigned channel, u32 data, u32 mem_mask)
{
	if (ACCESSING_BITS_0_7)
	{
		u8 const character = data;
		m_terminal->write(character);
		logerror(
				"UART%c TX: %02x %c\n",
				'A' + channel,
				character,
				(character >= 0x20 && character < 0x7f) ? character : '.');
	}
}


void datarover_state::terminal_key(u8 data)
{
	// The IDT monitor uses UART A by default.
	m_uart_rx_data[0] = data;
	m_uart_rx_ready[0] = true;
}


void datarover_state::betty_command(u32 command, bool subframe1)
{
	unsigned const reg = BIT(command, 27, 4);
	bool const write = BIT(command, 26);

	if (write && reg != 12)
		m_betty[reg] = u16(command);

	m_dino[subframe1 ? DINO_SIB_SF1_STATUS : DINO_SIB_SF0_STATUS] = m_betty[reg];
	m_dino[DINO_INTERRUPT1] |= subframe1 ? 0x00000080 : 0x00000100;
}


u32 datarover_state::uart_interrupt_r() const
{
	// The byte-wide holding registers are never back-pressured in this
	// skeleton, so both transmitters remain ready and empty.
	u32 result = m_dino[DINO_INTERRUPT2] | 0x0501'4000;
	if (m_uart_rx_ready[0])
		result |= 0x80000000;
	if (m_uart_rx_ready[1])
		result |= 0x00200000;

	return result;
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

	case DINO_RTC_HIGH:
	{
		u64 const ticks = BIT(m_dino[DINO_TIMER_CONTROL], 3)
				? 0
				: u64(machine().time().as_ticks(32'768)) - m_rtc_origin;
		return u32(ticks >> 32) & 0xff;
	}

	case DINO_RTC_LOW:
	{
		u64 const ticks = BIT(m_dino[DINO_TIMER_CONTROL], 3)
				? 0
				: u64(machine().time().as_ticks(32'768)) - m_rtc_origin;
		return u32(ticks);
	}

	case DINO_IO_CONTROL:
		// Input bits are sampled independently of the writable GPIO fields.
		return (m_dino[offset] & ~0x0000'0008) | m_boot_mode->read();

	case DINO_POWER_CONTROL:
		// Power-good is a read-only status input.  Without it, the low-level
		// boot path immediately invokes CommonShutdown and restarts forever.
		return m_dino[offset] | 0x2000'0000;

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
		COMBINE_DATA(&m_dino[offset]);
		if (BIT(m_dino[offset], 0))
			m_dino[DINO_INTERRUPT1] |= 0x0000'0580;
		break;

	case DINO_MBUS_CONTROL1:
		COMBINE_DATA(&m_dino[offset]);

		// MagicBus transfers are synchronous for now.  The monitor clears the
		// transmit status, enables the engine, then polls this completion bit.
		if (BIT(m_dino[offset], 0))
			m_dino[DINO_INTERRUPT2] |= 0x00000800;
		break;

	case DINO_TIMER_CONTROL:
	{
		bool const clear_was_asserted = BIT(m_dino[offset], 3);
		COMBINE_DATA(&m_dino[offset]);
		if (!clear_was_asserted && BIT(m_dino[offset], 3))
			m_rtc_origin = machine().time().as_ticks(32'768);
		break;
	}

	case DINO_POWER_CONTROL:
		COMBINE_DATA(&m_dino[offset]);

		// The low-level Betty reset uses Dino's stop timer as a short,
		// polled delay.  Complete it synchronously until a scheduled timer
		// and CPU stop/wake-up behavior are implemented.
		if (BIT(m_dino[offset], 11))
			m_dino[DINO_INTERRUPT5] |= 0x1000'0000;
		break;

	case DINO_INTERRUPT1:
		m_dino[offset] &= ~(data & mem_mask);

		// SIB subframes run continuously once enabled.  Reassert a cleared
		// frame or sound-receive flag immediately; production code waits for
		// these boundaries before placing SF0/SF1 commands and boot-beep data.
		if (BIT(m_dino[DINO_SIB_CONTROL], 0))
			m_dino[offset] |= data & mem_mask & 0x0000'0580;
		break;

	case DINO_INTERRUPT2:
	case DINO_INTERRUPT3:
	case DINO_INTERRUPT4:
	case DINO_INTERRUPT5:
	case DINO_INTERRUPT6:
		// Dino interrupt status registers are write-to-clear.
		m_dino[offset] &= ~(data & mem_mask);
		break;

	default:
		COMBINE_DATA(&m_dino[offset]);
		break;
	}
}


u32 datarover_state::glacier1_r(offs_t offset, u32 mem_mask)
{
	(void)mem_mask;
	return m_glacier1[offset];
}


void datarover_state::glacier1_w(offs_t offset, u32 data, u32 mem_mask)
{
	COMBINE_DATA(&m_glacier1[offset]);
}


u32 datarover_state::glacier2_r(offs_t offset, u32 mem_mask)
{
	(void)mem_mask;
	return m_glacier2[offset];
}


void datarover_state::glacier2_w(offs_t offset, u32 data, u32 mem_mask)
{
	COMBINE_DATA(&m_glacier2[offset]);
}


void datarover_state::machine_start()
{
	save_item(NAME(m_dino));
	save_item(NAME(m_glacier1));
	save_item(NAME(m_glacier2));
	save_item(NAME(m_betty));
	save_item(NAME(m_uart_rx_data));
	save_item(NAME(m_uart_rx_ready));
	save_item(NAME(m_rtc_origin));
}


void datarover_state::machine_reset()
{
	m_dino.fill(0);
	m_glacier1.fill(0);
	m_glacier2.fill(0);
	m_betty.fill(0);
	m_uart_rx_data.fill(0);
	m_uart_rx_ready.fill(false);
	m_rtc_origin = 0;

	// `touch_init` accepts revision 0x1002 or 0x1003.
	m_betty[12] = 0x1002;

	// Powered-on cold-start state: power good, cold-start and VCC enabled.
	m_dino[DINO_POWER_CONTROL] = 0x2000'0005;

	// PC Card CD1/CD2 are active low.  Keep both high until slots are
	// implemented so the monitor does not probe an absent attribute ROM.
	// Glacier exposes 16-bit registers on the upper half of each 32-bit word.
	m_glacier1[GLACIER_IO_DATA_INPUT] = 0x0c00'0000;
	m_glacier2[GLACIER_IO_DATA_INPUT] = 0x0c00'0000;
}


static INPUT_PORTS_START(datarover840)
	PORT_START("BOOT_MODE")
	PORT_CONFNAME(0x08, 0x08, "Power-on mode")
	PORT_CONFSETTING(0x08, "Magic Cap")
	PORT_CONFSETTING(0x00, "IDT monitor")
INPUT_PORTS_END


void datarover_state::datarover840(machine_config &config)
{
	// Keep the handheld LCD as the initial view.  The generic terminal is a
	// useful monitor/debugging surface, but normal Magic Cap boot leaves it
	// blank and its black raster is easily mistaken for a failed LCD.
	config.set_default_layout(layout_datarover840);

	R3900(config, m_maincpu, 36.864_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &datarover_state::memory_map);

	SCREEN(config, m_screen, SCREEN_TYPE_LCD);
	m_screen->set_refresh_hz(60);
	m_screen->set_size(480, 320);
	m_screen->set_visarea_full();
	m_screen->set_screen_update(FUNC(datarover_state::screen_update));

	GENERIC_TERMINAL(config, m_terminal);
	m_terminal->set_keyboard_callback(FUNC(datarover_state::terminal_key));
}


ROM_START(datarover840)
	ROM_REGION32_BE(0x800000, "maincpu", ROMREGION_ERASEFF)
	ROM_LOAD("magiccap-usa.image", 0x000000, 0x451817, CRC(0f9192ed) SHA1(4fc33eb40db75a0930861df383b109e46ae4ad91))
ROM_END

} // anonymous namespace


//    YEAR  NAME          PARENT  COMPAT  MACHINE       INPUT         CLASS             INIT        COMPANY          FULLNAME        FLAGS
COMP( 1998, datarover840, 0,      0,      datarover840, datarover840, datarover_state, empty_init, "General Magic", "DataRover 840", MACHINE_NO_SOUND | MACHINE_NOT_WORKING | MACHINE_SUPPORTS_SAVE )
