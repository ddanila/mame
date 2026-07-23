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
      - connect sound, modem and PC Card slots

***************************************************************************/

#include "emu.h"

#include "cpu/mips/mips1.h"
#include "machine/nvram.h"
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
		, m_rtc_nvram(*this, "rtc")
		, m_ram(*this, "ram")
		, m_rom(*this, "maincpu")
		, m_boot_mode(*this, "BOOT_MODE")
		, m_touch_x(*this, "TOUCH_X")
		, m_touch_y(*this, "TOUCH_Y")
		, m_touch_button(*this, "TOUCH_BUTTON")
		, m_power_button(*this, "POWER_BUTTON")
	{
	}

	void datarover840(machine_config &config);
	INPUT_CHANGED_MEMBER(touch_changed);
	INPUT_CHANGED_MEMBER(power_changed);

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
	static constexpr u32 DINO_INTERRUPT1_ENABLE = 0x118 / 4;
	static constexpr u32 DINO_INTERRUPT6_ENABLE = 0x12c / 4;
	static constexpr u32 DINO_RTC_HIGH = 0x140 / 4;
	static constexpr u32 DINO_RTC_LOW = 0x144 / 4;
	static constexpr u32 DINO_ALARM_HIGH = 0x148 / 4;
	static constexpr u32 DINO_ALARM_LOW = 0x14c / 4;
	static constexpr u32 DINO_TIMER_CONTROL = 0x150 / 4;
	static constexpr u32 DINO_PERIODIC_TIMER = 0x154 / 4;
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
	u16 touch_adc_value() const;
	u32 uart_interrupt_r() const;
	u32 uart_control_r(unsigned channel) const;
	u32 uart_hold_r(unsigned channel);
	void uart_hold_w(unsigned channel, u32 data, u32 mem_mask);
	void terminal_key(u8 data);
	void update_irq();
	void update_periodic_timer();
	u64 rtc_ticks() const;
	void persist_rtc();
	void update_rtc_timers();
	TIMER_CALLBACK_MEMBER(periodic_tick);
	TIMER_CALLBACK_MEMBER(rtc_alarm);
	TIMER_CALLBACK_MEMBER(rtc_rollover);
	TIMER_CALLBACK_MEMBER(rtc_persist_tick);
	TIMER_CALLBACK_MEMBER(sib_tick);
	u32 screen_update(screen_device &screen, bitmap_rgb32 &bitmap, rectangle const &cliprect);

	required_device<r3900_device> m_maincpu;
	required_device<screen_device> m_screen;
	required_device<generic_terminal_device> m_terminal;
	required_device<nvram_device> m_rtc_nvram;
	required_shared_ptr<u32> m_ram;
	required_region_ptr<u32> m_rom;
	required_ioport m_boot_mode;
	required_ioport m_touch_x;
	required_ioport m_touch_y;
	required_ioport m_touch_button;
	required_ioport m_power_button;

	std::array<u32, 0x200 / 4> m_dino{};
	std::array<u32, 0x24 / 4> m_glacier1{};
	std::array<u32, 0x24 / 4> m_glacier2{};
	std::array<u16, 16> m_betty{};
	std::array<u8, 2> m_uart_rx_data{};
	std::array<bool, 2> m_uart_rx_ready{};
	std::array<u32, 5> m_rtc_nvram_data{};
	u64 m_rtc_base = 0;
	u64 m_rtc_origin = 0;
	emu_timer *m_periodic_timer = nullptr;
	emu_timer *m_rtc_alarm_timer = nullptr;
	emu_timer *m_rtc_rollover_timer = nullptr;
	emu_timer *m_rtc_persist_timer = nullptr;
	emu_timer *m_sib_timer = nullptr;
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
	update_irq();
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
	update_irq();
}


void datarover_state::betty_command(u32 command, bool subframe1)
{
	unsigned const reg = BIT(command, 27, 4);
	bool const write = BIT(command, 26);

	if (write && reg != 12)
	{
		switch (reg)
		{
		case 0:
			// Betty IOData mixes writable outputs with sampled inputs.  Keep
			// the touchscreen contact bit under the pen input's control.
			m_betty[reg] = (u16(command) & ~0x1000) | (m_betty[reg] & 0x1000);
			break;

		default:
			m_betty[reg] = u16(command);
			break;
		}

		if (reg == 10)
			m_betty[11] = 0x8000 | ((touch_adc_value() & 0x03ff) << 5);
	}

	m_dino[subframe1 ? DINO_SIB_SF1_STATUS : DINO_SIB_SF0_STATUS] = m_betty[reg];
	m_dino[DINO_INTERRUPT1] |= subframe1 ? 0x00000080 : 0x00000100;
	update_irq();
}


u16 datarover_state::touch_adc_value() const
{
	// The power servers use Betty ADC inputs 24 and 28 for the main and
	// backup batteries.  Supply nominal healthy readings between the
	// Apollo calibration table's low/full thresholds.
	switch (m_betty[10] & 0x001d)
	{
	case 0x18:
		return 800;

	case 0x1c:
		return 340;
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

	if (newval)
	{
		m_betty[0] |= TOUCH_MASK;
		if (m_betty[2] & TOUCH_MASK)
		{
			m_dino[DINO_SIB_CONTROL] |= 0x8000'0000;
			m_dino[DINO_INTERRUPT1] |= 0x0000'0040;
		}
	}
	else
	{
		m_betty[0] &= ~TOUCH_MASK;
		if (m_betty[3] & TOUCH_MASK)
		{
			m_dino[DINO_SIB_CONTROL] |= 0x8000'0000;
			m_dino[DINO_INTERRUPT1] |= 0x0000'0020;
		}
	}

	update_irq();
}


INPUT_CHANGED_MEMBER(datarover_state::power_changed)
{
	if (newval)
	{
		if (m_maincpu->suspended(SUSPEND_REASON_HALT))
		{
			// Dino's on-button is also the wake source.  Hardware restores
			// VCC and releases StopCpu before presenting the edge interrupt.
			// Betty is on the switched peripheral rail, so wake begins with
			// its register file and Dino SIB handshake state reset.
			m_betty.fill(0);
			m_betty[12] = 0x1002;
			m_dino[DINO_SIB_CONTROL] = 0;
			m_dino[DINO_SIB_SF0_STATUS] = 0;
			m_dino[DINO_SIB_SF1_STATUS] = 0;
			m_dino[DINO_INTERRUPT1] &= ~0x0000'05e0;
			m_sib_timer->reset();
			m_dino[DINO_POWER_CONTROL] |= 0x0000'0001;
			m_dino[DINO_POWER_CONTROL] &= ~0x0000'0010;
			m_maincpu->resume(SUSPEND_REASON_HALT);
		}
		m_dino[DINO_POWER_CONTROL] |= 0x8000'0000;
		m_dino[DINO_INTERRUPT5] |= 0x0080'0000;
	}
	else
	{
		m_dino[DINO_POWER_CONTROL] &= ~0x8000'0000;
		m_dino[DINO_INTERRUPT5] |= 0x0040'0000;
	}

	update_irq();
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

void datarover_state::update_irq()
{
	bool pending = false;

	if (BIT(m_dino[DINO_INTERRUPT6_ENABLE], 18))
	{
		for (unsigned bank = 0; bank < 5; ++bank)
		{
			u32 const status = (bank == 1)
					? uart_interrupt_r()
					: m_dino[DINO_INTERRUPT1 + bank];
			pending |= bool(status & m_dino[DINO_INTERRUPT1_ENABLE + bank]);
		}
	}

	// Dino's general interrupt is IP4 (Cause bit 12), MIPS input line 2.
	m_maincpu->set_input_line(2, pending ? ASSERT_LINE : CLEAR_LINE);
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
	if (BIT(m_dino[DINO_SIB_CONTROL], 0))
	{
		// Betty's serial bus continually produces frame boundaries and sound
		// receive slots.  Schedule them rather than reasserting immediately
		// on clear, allowing the interrupt handler to make forward progress.
		m_dino[DINO_INTERRUPT1] |= 0x0000'0580;
		update_irq();
	}
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
		return u32(rtc_ticks() >> 32) & 0xff;

	case DINO_RTC_LOW:
		return u32(rtc_ticks());

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
		{
			m_dino[DINO_INTERRUPT1] |= 0x0000'0580;
			m_sib_timer->adjust(attotime::from_msec(1), 0, attotime::from_msec(1));
		}
		else
		{
			m_sib_timer->reset();
		}
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

	case DINO_ALARM_HIGH:
	case DINO_ALARM_LOW:
		COMBINE_DATA(&m_dino[offset]);
		update_rtc_timers();
		break;

	case DINO_POWER_CONTROL:
	{
		u32 const old_control = m_dino[offset];
		u32 const status = m_dino[offset] & 0xe000'0000;
		COMBINE_DATA(&m_dino[offset]);
		m_dino[offset] = (m_dino[offset] & ~0xe000'0000) | status;

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
		else if (BIT(old_control, 0) && !BIT(m_dino[offset], 0)
				&& BIT(m_dino[offset], 4))
		{
			// The normal power-button path is suspend-to-RAM, not loss of
			// battery-backed state.  An on-button edge resumes this CPU.
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
	case DINO_INTERRUPT6:
		// Dino interrupt status registers are write-to-clear.
		m_dino[offset] &= ~(data & mem_mask);
		break;

	default:
		COMBINE_DATA(&m_dino[offset]);
		break;
	}

	update_irq();
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
	m_periodic_timer = timer_alloc(FUNC(datarover_state::periodic_tick), this);
	m_rtc_alarm_timer = timer_alloc(FUNC(datarover_state::rtc_alarm), this);
	m_rtc_rollover_timer = timer_alloc(FUNC(datarover_state::rtc_rollover), this);
	m_rtc_persist_timer = timer_alloc(FUNC(datarover_state::rtc_persist_tick), this);
	m_sib_timer = timer_alloc(FUNC(datarover_state::sib_tick), this);
	m_rtc_nvram->set_base(m_rtc_nvram_data.data(), sizeof(m_rtc_nvram_data));

	save_item(NAME(m_dino));
	save_item(NAME(m_glacier1));
	save_item(NAME(m_glacier2));
	save_item(NAME(m_betty));
	save_item(NAME(m_uart_rx_data));
	save_item(NAME(m_uart_rx_ready));
	save_item(NAME(m_rtc_nvram_data));
	save_item(NAME(m_rtc_base));
	save_item(NAME(m_rtc_origin));
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
	m_glacier1.fill(0);
	m_glacier2.fill(0);
	m_betty.fill(0);
	m_uart_rx_data.fill(0);
	m_uart_rx_ready.fill(false);
	m_periodic_timer->reset();
	m_rtc_alarm_timer->reset();
	m_rtc_rollover_timer->reset();
	m_rtc_persist_timer->adjust(attotime::from_seconds(1), 0, attotime::from_seconds(1));
	m_sib_timer->reset();

	static constexpr u32 RTC_NVRAM_SIGNATURE = 0x4452'5443; // "DRTC"
	static constexpr u64 RTC_MASK = 0x0000'00ff'ffff'ffff;
	m_rtc_base = 0;
	if (m_rtc_nvram_data[0] == RTC_NVRAM_SIGNATURE)
	{
		u64 const saved_ticks =
				(u64(m_rtc_nvram_data[1]) << 32) | m_rtc_nvram_data[2];
		u64 const saved_host_time =
				(u64(m_rtc_nvram_data[3]) << 32) | m_rtc_nvram_data[4];
		system_time now;
		machine().current_datetime(now);
		s64 const elapsed = now.time - s64(saved_host_time);
		m_rtc_base = (saved_ticks + (u64(std::max<s64>(elapsed, 0)) * 32'768)) & RTC_MASK;
	}
	m_rtc_origin = machine().time().as_ticks(32'768);

	// `touch_init` accepts revision 0x1002 or 0x1003.
	m_betty[12] = 0x1002;

	// Main DRAM is battery-backed.  A zero-filled new NVRAM image is a cold
	// start; loaded state retains the heap and follows Dino's warm-start path.
	m_dino[DINO_POWER_CONTROL] = retained_ram ? 0x2000'0001 : 0x2000'0005;

	// PC Card CD1/CD2 are active low.  Keep both high until slots are
	// implemented so the monitor does not probe an absent attribute ROM.
	// Glacier exposes 16-bit registers on the upper half of each 32-bit word.
	m_glacier1[GLACIER_IO_DATA_INPUT] = 0x0c00'0000;
	m_glacier2[GLACIER_IO_DATA_INPUT] = 0x0c00'0000;
	update_rtc_timers();
	update_irq();
}


static INPUT_PORTS_START(datarover840)
	PORT_START("BOOT_MODE")
	PORT_CONFNAME(0x08, 0x08, "Power-on mode")
	PORT_CONFSETTING(0x08, "Magic Cap")
	PORT_CONFSETTING(0x00, "IDT monitor")

	PORT_START("TOUCH_X")
	PORT_BIT(0xffff, 0x8000, IPT_LIGHTGUN_X) PORT_NAME("Pen X") PORT_MINMAX(0, 0xffff) PORT_SENSITIVITY(50) PORT_KEYDELTA(256) PORT_CROSSHAIR(X, 1.0, 0.0, 0)

	PORT_START("TOUCH_Y")
	PORT_BIT(0xffff, 0x8000, IPT_LIGHTGUN_Y) PORT_NAME("Pen Y") PORT_MINMAX(0, 0xffff) PORT_SENSITIVITY(50) PORT_KEYDELTA(256) PORT_CROSSHAIR(Y, 1.0, 0.0, 0)

	PORT_START("TOUCH_BUTTON")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_BUTTON1) PORT_NAME("Touch screen") PORT_CODE(MOUSECODE_BUTTON1) PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(datarover_state::touch_changed), 0)

	PORT_START("POWER_BUTTON")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Power button") PORT_CODE(KEYCODE_END) PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(datarover_state::power_changed), 0)
INPUT_PORTS_END


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
