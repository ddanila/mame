// license:BSD-3-Clause
// copyright-holders:Aaron Giles, Patrick Mackinlay

/*
 * MIPS-I emulation, including R2000[A], R3000[A] and IDT R30xx devices. The
 * IDT devices come in two variations: those with an "E" suffix include a TLB,
 * while those without have hard-wired address translation.
 *
 * TODO:
 *  - multi-word cache line sizes
 *  - R3041 features
 *
 */
#include "emu.h"
#include "mips1.h"
#include "mips1dsm.h"
#include "softfloat3/source/include/softfloat.h"

#define LOG_TLB     (1U << 1)

//#define VERBOSE     (LOG_GENERAL|LOG_TLB)

#include "logmacro.h"

enum registers : unsigned
{
	MIPS1_R0   = 0,
	MIPS1_COP0 = 32,
	MIPS1_F0   = 64,

	MIPS1_PC   = 80,
	MIPS1_HI,
	MIPS1_LO,
	MIPS1_FCR30,
	MIPS1_FCR31,
};

enum exception : u32
{
	EXCEPTION_INTERRUPT = 0x00000000,
	EXCEPTION_TLBMOD    = 0x00000004,
	EXCEPTION_TLBLOAD   = 0x00000008,
	EXCEPTION_TLBSTORE  = 0x0000000c,
	EXCEPTION_ADDRLOAD  = 0x00000010,
	EXCEPTION_ADDRSTORE = 0x00000014,
	EXCEPTION_BUSINST   = 0x00000018,
	EXCEPTION_BUSDATA   = 0x0000001c,
	EXCEPTION_SYSCALL   = 0x00000020,
	EXCEPTION_BREAK     = 0x00000024,
	EXCEPTION_INVALIDOP = 0x00000028,
	EXCEPTION_BADCOP    = 0x0000002c,
	EXCEPTION_OVERFLOW  = 0x00000030,
	EXCEPTION_TRAP      = 0x00000034,

	EXCEPTION_BADCOP0   = 0x0000002c,
	EXCEPTION_BADCOP1   = 0x1000002c,
	EXCEPTION_BADCOP2   = 0x2000002c,
	EXCEPTION_BADCOP3   = 0x3000002c,
};

constexpr u8 COP0_Index    = 0;
constexpr u8 COP0_Random   = 1;
constexpr u8 COP0_EntryLo  = 2;
constexpr u8 COP0_BusCtrl  = 2;  // r3041 only
constexpr u8 COP0_Config   = 3;  // r3041/r3071/r3081/r3900 only
constexpr u8 COP0_Context  = 4;
constexpr u8 COP0_Cache    = 7;  // r3900 only
constexpr u8 COP0_BadVAddr = 8;
constexpr u8 COP0_Count    = 9;  // r3041 only
constexpr u8 COP0_EntryHi  = 10;
constexpr u8 COP0_PortSize = 10; // r3041 only
constexpr u8 COP0_Compare  = 11; // r3041 only
constexpr u8 COP0_Status   = 12;
constexpr u8 COP0_Cause    = 13;
constexpr u8 COP0_EPC      = 14;
constexpr u8 COP0_PRId     = 15;
constexpr u8 COP0_Debug    = 16; // r3900 only
constexpr u8 COP0_DEPC     = 17; // r3900 only

enum sr_mask : u32
{
	SR_IEc    = 0x00000001, // interrupt enable (current)
	SR_KUc    = 0x00000002, // user mode (current)
	SR_IEp    = 0x00000004, // interrupt enable (previous)
	SR_KUp    = 0x00000008, // user mode (previous)
	SR_IEo    = 0x00000010, // interrupt enable (old)
	SR_KUo    = 0x00000020, // user mode (old)
	SR_IMSW0  = 0x00000100, // software interrupt 0 enable
	SR_IMSW1  = 0x00000200, // software interrupt 1 enable
	SR_IMEX0  = 0x00000400, // external interrupt 0 enable
	SR_IMEX1  = 0x00000800, // external interrupt 1 enable
	SR_IMEX2  = 0x00001000, // external interrupt 2 enable
	SR_IMEX3  = 0x00002000, // external interrupt 3 enable
	SR_IMEX4  = 0x00004000, // external interrupt 4 enable
	SR_IMEX5  = 0x00008000, // external interrupt 5 enable
	SR_IsC    = 0x00010000, // isolate (data) cache
	SR_SwC    = 0x00020000, // swap caches
	SR_PZ     = 0x00040000, // cache parity zero
	SR_CM     = 0x00080000, // cache miss
	SR_PE     = 0x00100000, // cache parity error
	SR_TS     = 0x00200000, // tlb shutdown
	SR_BEV    = 0x00400000, // boot exception vectors
	SR_RE     = 0x02000000, // reverse endianness in user mode
	SR_COP0   = 0x10000000, // coprocessor 0 usable
	SR_COP1   = 0x20000000, // coprocessor 1 usable
	SR_COP2   = 0x40000000, // coprocessor 2 usable
	SR_COP3   = 0x80000000, // coprocessor 3 usable

	SR_KUIE   = 0x0000003f, // all interrupt enable and user mode bits
	SR_KUIEpc = 0x0000000f, // previous and current interrupt enable and user mode bits
	SR_KUIEop = 0x0000003c, // old and previous interrupt enable and user mode bits
	SR_IM     = 0x0000ff00, // all interrupt mask bits
};

enum cause_mask : u32
{
	CAUSE_EXCCODE = 0x0000007c, // exception code
	CAUSE_IPSW0   = 0x00000100, // software interrupt 0 pending
	CAUSE_IPSW1   = 0x00000200, // software interrupt 1 pending
	CAUSE_IPEX0   = 0x00000400, // external interrupt 0 pending
	CAUSE_IPEX1   = 0x00000800, // external interrupt 1 pending
	CAUSE_IPEX2   = 0x00001000, // external interrupt 2 pending
	CAUSE_IPEX3   = 0x00002000, // external interrupt 3 pending
	CAUSE_IPEX4   = 0x00004000, // external interrupt 4 pending
	CAUSE_IPEX5   = 0x00008000, // external interrupt 5 pending
	CAUSE_IP      = 0x0000ff00, // interrupt pending
	CAUSE_CE      = 0x30000000, // co-processor error
	CAUSE_BD      = 0x80000000, // branch delay

	CAUSE_IPEX    = 0x0000fc00, // external interrupt pending
};

enum debug_mask : u32
{
	DEBUG_DBD = 0x8000'0000, // debug branch delay
	DEBUG_DM  = 0x4000'0000, // debug mode
	DEBUG_BSF = 0x0000'0400, // bus error exception flag
	DEBUG_SST = 0x0000'0100, // single step enable
	DEBUG_DBP = 0x0000'0002, // debug breakpoint
	DEBUG_DSS = 0x0000'0001, // debug single step
};

enum entryhi_mask : u32
{
	EH_VPN  = 0xfffff000, // virtual page number
	EH_ASID = 0x00000fc0, // address space identifier

	EH_WM   = 0xffffffc0, // write mask
};
enum entrylo_mask : u32
{
	EL_PFN = 0xfffff000, // physical frame
	EL_N   = 0x00000800, // noncacheable
	EL_D   = 0x00000400, // dirty
	EL_V   = 0x00000200, // valid
	EL_G   = 0x00000100, // global

	EL_WM  = 0xffffff00, // write mask
};
enum context_mask : u32
{
	PTE_BASE = 0xffe00000, // base address of page table
	BAD_VPN  = 0x001ffffc, // virtual address bits 30..12
};

enum cp1_fcr31_mask : u32
{
	FCR31_RM = 0x00000003, // rounding mode

	FCR31_FI = 0x00000004, // inexact operation flag
	FCR31_FU = 0x00000008, // underflow flag
	FCR31_FO = 0x00000010, // overflow flag
	FCR31_FZ = 0x00000020, // divide by zero flag
	FCR31_FV = 0x00000040, // invalid operation flag

	FCR31_EI = 0x00000080, // inexact operation enable
	FCR31_EU = 0x00000100, // underflow enable
	FCR31_EO = 0x00000200, // overflow enable
	FCR31_EZ = 0x00000400, // divide by zero enable
	FCR31_EV = 0x00000800, // invalid operation enable

	FCR31_CI = 0x00001000, // inexact operation cause
	FCR31_CU = 0x00002000, // underflow cause
	FCR31_CO = 0x00004000, // overflow cause
	FCR31_CZ = 0x00008000, // divide by zero cause
	FCR31_CV = 0x00010000, // invalid operation cause
	FCR31_CE = 0x00020000, // unimplemented operation cause

	FCR31_C = 0x00800000, // condition

	FCR31_FM = 0x0000007c, // flag mask
	FCR31_EM = 0x00000f80, // enable mask
	FCR31_CM = 0x0001f000, // cause mask (except unimplemented)
};

#define RSREG           ((op >> 21) & 31)
#define RTREG           ((op >> 16) & 31)
#define RDREG           ((op >> 11) & 31)
#define SHIFT           ((op >> 6) & 31)

#define FTREG           ((op >> 16) & 31)
#define FSREG           ((op >> 11) & 31)
#define FDREG           ((op >> 6) & 31)

#define SIMMVAL         s16(op)
#define UIMMVAL         u16(op)
#define LIMMVAL         (op & 0x03ffffff)

#define SR              m_cop0[COP0_Status]
#define CAUSE           m_cop0[COP0_Cause]

DEFINE_DEVICE_TYPE(R2000,       r2000_device,     "r2000",   "MIPS R2000")
DEFINE_DEVICE_TYPE(R2000A,      r2000a_device,    "r2000a",  "MIPS R2000A")
DEFINE_DEVICE_TYPE(R3000,       r3000_device,     "r3000",   "MIPS R3000")
DEFINE_DEVICE_TYPE(R3000A,      r3000a_device,    "r3000a",  "MIPS R3000A")
DEFINE_DEVICE_TYPE(R3041,       r3041_device,     "r3041",   "IDT R3041")
DEFINE_DEVICE_TYPE(R3051,       r3051_device,     "r3051",   "IDT R3051")
DEFINE_DEVICE_TYPE(R3052,       r3052_device,     "r3052",   "IDT R3052")
DEFINE_DEVICE_TYPE(R3052E,      r3052e_device,    "r3052e",  "IDT R3052E")
DEFINE_DEVICE_TYPE(R3071,       r3071_device,     "r3071",   "IDT R3071")
DEFINE_DEVICE_TYPE(R3081,       r3081_device,     "r3081",   "IDT R3081")
DEFINE_DEVICE_TYPE(R3900,       r3900_device,     "r3900",    "Toshiba R3900")
DEFINE_DEVICE_TYPE(SONYPS2_IOP, iop_device,       "sonyiop", "Sony Playstation 2 IOP")

ALLOW_SAVE_TYPE(mips1core_device_base::branch_state);

mips1core_device_base::mips1core_device_base(machine_config const &mconfig, device_type type, char const *tag, device_t *owner, u32 clock, u32 cpurev, size_t icache_size, size_t dcache_size, bool cache_pws, bool multiply_to_gpr, unsigned dcache_ways)
	: cpu_device(mconfig, type, tag, owner, clock)
	, m_program_config_be("program", ENDIANNESS_BIG, 32, 32)
	, m_program_config_le("program", ENDIANNESS_LITTLE, 32, 32)
	, m_cpurev(cpurev)
	, m_endianness(ENDIANNESS_BIG)
	, m_multiply_to_gpr(multiply_to_gpr)
	, m_divide_hi(0)
	, m_divide_lo(0)
	, m_divide_cycles(0)
	, m_gpr_delay(0)
	, m_debug_step_suppress(false)
	, m_icount(0)
	, m_icache(icache_size)
	, m_dcache(dcache_size, dcache_ways)
	, m_cache((icache_size && dcache_size) ? CACHED : UNCACHED)
	, m_cache_pws(cache_pws)
	, m_in_brcond(*this, 0)
{
}

mips1_device_base::mips1_device_base(machine_config const &mconfig, device_type type, char const *tag, device_t *owner, u32 clock, u32 cpurev, size_t icache_size, size_t dcache_size, bool cache_pws)
	: mips1core_device_base(mconfig, type, tag, owner, clock, cpurev, icache_size, dcache_size, cache_pws)
	, m_fcr0(0)
{
}

r2000_device::r2000_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock, size_t icache_size, size_t dcache_size)
	: mips1_device_base(mconfig, R2000, tag, owner, clock, 0x0120, icache_size, dcache_size, false)
{
}

r2000a_device::r2000a_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock, size_t icache_size, size_t dcache_size)
	: mips1_device_base(mconfig, R2000A, tag, owner, clock, 0x0210, icache_size, dcache_size, false)
{
}

r3000_device::r3000_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock, size_t icache_size, size_t dcache_size)
	: mips1_device_base(mconfig, R3000, tag, owner, clock, 0x0220, icache_size, dcache_size, false)
{
}

r3000a_device::r3000a_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock, size_t icache_size, size_t dcache_size)
	: mips1_device_base(mconfig, R3000A, tag, owner, clock, 0x0230, icache_size, dcache_size, false)
{
}

r3041_device::r3041_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock)
	: mips1core_device_base(mconfig, R3041, tag, owner, clock, 0x0700, 2048, 512, true)
{
}

r3051_device::r3051_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock)
	: mips1core_device_base(mconfig, R3051, tag, owner, clock, 0x0200, 4096, 2048, true)
{
}

r3052_device::r3052_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock)
	: mips1core_device_base(mconfig, R3052, tag, owner, clock, 0x0200, 8192, 2048, true)
{
}

r3052e_device::r3052e_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock)
	: mips1_device_base(mconfig, R3052E, tag, owner, clock, 0x0200, 8192, 2048, true)
{
}

r3071_device::r3071_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock, size_t icache_size, size_t dcache_size)
	: mips1_device_base(mconfig, R3071, tag, owner, clock, 0x0200, icache_size, dcache_size, true)
{
}

r3081_device::r3081_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock, size_t icache_size, size_t dcache_size)
	: mips1_device_base(mconfig, R3081, tag, owner, clock, 0x0200, icache_size, dcache_size, true)
{
	set_fpu(0x0300);
}

r3900_device::r3900_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock)
	: mips1core_device_base(mconfig, R3900, tag, owner, clock, 0x2200, 4096, 1024, true, true, 2)
{
}

void r3900_device::device_start()
{
	mips1core_device_base::device_start();

	state_add(MIPS1_COP0 + COP0_Config, "Config", m_cop0[COP0_Config]);
	state_add(MIPS1_COP0 + COP0_Cache, "Cache", m_cop0[COP0_Cache]);
	state_add(MIPS1_COP0 + COP0_Debug, "Debug", m_cop0[COP0_Debug]);
	state_add(MIPS1_COP0 + COP0_DEPC, "DEPC", m_cop0[COP0_DEPC]);
}

void r3900_device::device_reset()
{
	mips1core_device_base::device_reset();

	// The TMPR3902U has a 4 KiB instruction cache and 1 KiB data cache.
	// Both caches are enabled after reset; all other writable fields and
	// all cache auto-lock modes are clear.
	m_cop0[COP0_Config] = 0x0010'0030;
	m_cop0[COP0_Cache] = 0;
	m_cop0[COP0_Debug] = 0;
	m_cop0[COP0_DEPC] = 0;
	set_clock_scale(1.0);
}

iop_device::iop_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock)
	: mips1core_device_base(mconfig, SONYPS2_IOP, tag, owner, clock, 0x001f, 4096, 1024, false)
{
	m_endianness = ENDIANNESS_LITTLE;
}

void mips1core_device_base::device_start()
{
	// set our instruction counter
	set_icountptr(m_icount);

	// register our state for the debugger
	state_add(STATE_GENPC,      "GENPC",     m_pc).callimport().noshow();
	state_add(STATE_GENPCBASE,  "CURPC",     m_pc).noshow();

	state_add(MIPS1_PC,                   "PC",        m_pc).callimport();
	state_add(MIPS1_COP0 + COP0_Status,   "SR",        m_cop0[COP0_Status]);

	for (unsigned i = 0; i < std::size(m_r); i++)
		state_add(MIPS1_R0 + i, util::string_format("R%d", i).c_str(), m_r[i]);

	state_add(MIPS1_HI, "HI", m_hi);
	state_add(MIPS1_LO, "LO", m_lo);

	// cop0 exception registers
	state_add(MIPS1_COP0 + COP0_BadVAddr, "BadVAddr", m_cop0[COP0_BadVAddr]);
	state_add(MIPS1_COP0 + COP0_Cause,    "Cause",    m_cop0[COP0_Cause]);
	state_add(MIPS1_COP0 + COP0_EPC,      "EPC",      m_cop0[COP0_EPC]);

	// register our state for saving
	save_item(NAME(m_pc));
	save_item(NAME(m_hi));
	save_item(NAME(m_lo));
	save_item(NAME(m_divide_hi));
	save_item(NAME(m_divide_lo));
	save_item(NAME(m_divide_cycles));
	save_item(NAME(m_gpr_delay));
	save_item(NAME(m_debug_step_suppress));
	save_item(NAME(m_r));
	save_item(NAME(m_cop0));
	save_item(NAME(m_branch_state));
	save_item(NAME(m_branch_target));

	// initialise cpu id register
	m_cop0[COP0_PRId] = m_cpurev;

	m_cop0[COP0_Cause] = 0;

	m_r[0] = 0;

	m_icache.start();
	m_dcache.start();

	save_pointer(STRUCT_MEMBER(m_icache.line, tag), m_icache.lines());
	save_pointer(STRUCT_MEMBER(m_icache.line, data), m_icache.lines());
	save_pointer(STRUCT_MEMBER(m_icache.line, locked), m_icache.lines());
	save_pointer(NAME(m_icache.lru), m_icache.sets());
	save_pointer(STRUCT_MEMBER(m_dcache.line, tag), m_dcache.lines());
	save_pointer(STRUCT_MEMBER(m_dcache.line, data), m_dcache.lines());
	save_pointer(STRUCT_MEMBER(m_dcache.line, locked), m_dcache.lines());
	save_pointer(NAME(m_dcache.lru), m_dcache.sets());
}

void mips1core_device_base::state_import(device_state_entry const &entry)
{
	if (entry.index() == STATE_GENPC || entry.index() == MIPS1_PC)
	{
		m_branch_state = NONE;
		m_branch_target = 0;
		m_debug_step_suppress = false;
	}
}

void r3041_device::device_start()
{
	mips1core_device_base::device_start();

	// cop0 r3041 registers
	state_add(MIPS1_COP0 + COP0_BusCtrl,  "BusCtrl", m_cop0[COP0_BusCtrl]);
	state_add(MIPS1_COP0 + COP0_Config,   "Config", m_cop0[COP0_Config]);
	state_add(MIPS1_COP0 + COP0_Count,    "Count", m_cop0[COP0_Count]);
	state_add(MIPS1_COP0 + COP0_PortSize, "PortSize", m_cop0[COP0_PortSize]);
	state_add(MIPS1_COP0 + COP0_Compare,  "Compare", m_cop0[COP0_Compare]);

	m_cop0[COP0_BusCtrl] = 0x20130b00U;
	m_cop0[COP0_Config] = 0x40000000U;
	m_cop0[COP0_PortSize] = 0;
}

void mips1core_device_base::device_reset()
{
	// initialize the state
	m_pc = 0xbfc00000;
	m_branch_state = NONE;
	m_divide_cycles = 0;
	m_gpr_delay = 0;
	m_debug_step_suppress = false;
	m_icache.reset();
	m_dcache.reset();

	// non-tlb devices have tlb shut down
	m_cop0[COP0_Status] = SR_BEV | SR_TS;

	m_bus_error = false;
}

void r3041_device::device_reset()
{
	mips1core_device_base::device_reset();

	m_cop0[COP0_Count] = 0;
	m_cop0[COP0_Compare] = 0x00ffffffU;
}

void mips1core_device_base::execute_run()
{
	// core execution loop
	while (m_icount-- > 0)
	{
		int const cycle_start = m_icount;
		bool divide_started = false;
		bool const debug_step_suppressed = m_debug_step_suppress;

		// debugging
		debugger_instruction_hook(m_pc);

		if (m_multiply_to_gpr
				&& !debug_step_suppressed
				&& (m_branch_state != DELAY)
				&& (m_cop0[COP0_Debug] & DEBUG_SST)
				&& !(m_cop0[COP0_Debug] & DEBUG_DM))
		{
			generate_debug_exception(DEBUG_DSS);
		}
		else
		{
			// fetch instruction
			fetch(m_pc, [this, &divide_started](u32 const op)
			{
			// check for interrupts
			if ((CAUSE & SR & SR_IM) && (SR & SR_IEc))
			{
				// enable debugger interrupt breakpoints
				for (int irqline = 0; irqline < 6; irqline++)
				{
					if (CAUSE & SR & (CAUSE_IPEX0 << irqline))
					{
						standard_irq_callback(irqline, m_pc);
						break;
					}
				}
				generate_exception(EXCEPTION_INTERRUPT);
				return;
			}

			// A TX39 load or three-operand multiply writes its GPR one pipeline
			// stage later.  Only an immediately following dependent instruction
			// stalls; independent instructions continue at one per cycle.
			gpr_interlock(op);

			// decode and execute instruction
			switch (op >> 26)
			{
			case 0x00: // SPECIAL
				switch (op & 63)
				{
				case 0x00: // SLL
					m_r[RDREG] = m_r[RTREG] << SHIFT;
					break;
				case 0x02: // SRL
					m_r[RDREG] = m_r[RTREG] >> SHIFT;
					break;
				case 0x03: // SRA
					m_r[RDREG] = s32(m_r[RTREG]) >> SHIFT;
					break;
				case 0x04: // SLLV
					m_r[RDREG] = m_r[RTREG] << (m_r[RSREG] & 31);
					break;
				case 0x06: // SRLV
					m_r[RDREG] = m_r[RTREG] >> (m_r[RSREG] & 31);
					break;
				case 0x07: // SRAV
					m_r[RDREG] = s32(m_r[RTREG]) >> (m_r[RSREG] & 31);
					break;
				case 0x08: // JR
					m_branch_state = BRANCH;
					m_branch_target = m_r[RSREG];
					break;
				case 0x09: // JALR
					m_branch_state = BRANCH;
					m_branch_target = m_r[RSREG];
					m_r[RDREG] = m_pc + 8;
					break;
				case 0x0c: // SYSCALL
					generate_exception(EXCEPTION_SYSCALL);
					break;
				case 0x0d: // BREAK
					generate_exception(EXCEPTION_BREAK);
					break;
				case 0x0e: // R3900 SDBBP
					if (m_multiply_to_gpr)
						generate_debug_exception(DEBUG_DBP);
					else
						generate_exception(EXCEPTION_INVALIDOP);
					break;
				case 0x0f: // R3900 SYNC
					// Memory accesses and cache refills are synchronous in
					// this interpreter, so there is nothing left to drain.
					if (!m_multiply_to_gpr || (op & 0x03ff'ffc0))
						generate_exception(EXCEPTION_INVALIDOP);
					break;
				case 0x10: // MFHI
					divide_interlock();
					m_r[RDREG] = m_hi;
					break;
				case 0x11: // MTHI
					cancel_divide();
					m_hi = m_r[RSREG];
					break;
				case 0x12: // MFLO
					divide_interlock();
					m_r[RDREG] = m_lo;
					break;
				case 0x13: // MTLO
					cancel_divide();
					m_lo = m_r[RSREG];
					break;
				case 0x18: // MULT
					{
						u64 product = mul_32x32(m_r[RSREG], m_r[RTREG]);

						m_lo = product;
						m_hi = product >> 32;
						if (m_multiply_to_gpr)
						{
							m_r[RDREG] = m_lo;
							set_gpr_delay(RDREG);
						}
						else
							m_icount -= 11;
					}
					break;
				case 0x19: // MULTU
					{
						u64 product = mulu_32x32(m_r[RSREG], m_r[RTREG]);

						m_lo = product;
						m_hi = product >> 32;
						if (m_multiply_to_gpr)
						{
							m_r[RDREG] = m_lo;
							set_gpr_delay(RDREG);
						}
						else
							m_icount -= 11;
					}
					break;
				case 0x1a: // DIV
					if (m_multiply_to_gpr)
					{
						cancel_divide();
						m_divide_lo = m_lo;
						m_divide_hi = m_hi;
						if (m_r[RTREG])
						{
							if (m_r[RSREG] == 0x8000'0000U
									&& m_r[RTREG] == 0xffff'ffffU)
							{
								m_divide_lo = 0x8000'0000U;
								m_divide_hi = 0;
							}
							else
							{
								m_divide_lo =
										s32(m_r[RSREG]) / s32(m_r[RTREG]);
								m_divide_hi =
										s32(m_r[RSREG]) % s32(m_r[RTREG]);
							}
						}
						m_divide_cycles = 35;
						divide_started = true;
					}
					else
					{
						if (m_r[RTREG])
						{
							m_lo = s32(m_r[RSREG]) / s32(m_r[RTREG]);
							m_hi = s32(m_r[RSREG]) % s32(m_r[RTREG]);
						}
						m_icount -= 34;
					}
					break;
				case 0x1b: // DIVU
					if (m_multiply_to_gpr)
					{
						cancel_divide();
						m_divide_lo = m_lo;
						m_divide_hi = m_hi;
						if (m_r[RTREG])
						{
							m_divide_lo = m_r[RSREG] / m_r[RTREG];
							m_divide_hi = m_r[RSREG] % m_r[RTREG];
						}
						m_divide_cycles = 35;
						divide_started = true;
					}
					else
					{
						if (m_r[RTREG])
						{
							m_lo = m_r[RSREG] / m_r[RTREG];
							m_hi = m_r[RSREG] % m_r[RTREG];
						}
						m_icount -= 34;
					}
					break;
				case 0x20: // ADD
					{
						u32 const sum = m_r[RSREG] + m_r[RTREG];

						// overflow: (sign(addend0) == sign(addend1)) && (sign(addend0) != sign(sum))
						if (!BIT(m_r[RSREG] ^ m_r[RTREG], 31) && BIT(m_r[RSREG] ^ sum, 31))
							generate_exception(EXCEPTION_OVERFLOW);
						else
							m_r[RDREG] = sum;
					}
					break;
				case 0x21: // ADDU
					m_r[RDREG] = m_r[RSREG] + m_r[RTREG];
					break;
				case 0x22: // SUB
					{
						u32 const difference = m_r[RSREG] - m_r[RTREG];

						// overflow: (sign(minuend) != sign(subtrahend)) && (sign(minuend) != sign(difference))
						if (BIT(m_r[RSREG] ^ m_r[RTREG], 31) && BIT(m_r[RSREG] ^ difference, 31))
							generate_exception(EXCEPTION_OVERFLOW);
						else
							m_r[RDREG] = difference;
					}
					break;
				case 0x23: // SUBU
					m_r[RDREG] = m_r[RSREG] - m_r[RTREG];
					break;
				case 0x24: // AND
					m_r[RDREG] = m_r[RSREG] & m_r[RTREG];
					break;
				case 0x25: // OR
					m_r[RDREG] = m_r[RSREG] | m_r[RTREG];
					break;
				case 0x26: // XOR
					m_r[RDREG] = m_r[RSREG] ^ m_r[RTREG];
					break;
				case 0x27: // NOR
					m_r[RDREG] = ~(m_r[RSREG] | m_r[RTREG]);
					break;
				case 0x2a: // SLT
					m_r[RDREG] = s32(m_r[RSREG]) < s32(m_r[RTREG]);
					break;
				case 0x2b: // SLTU
					m_r[RDREG] = u32(m_r[RSREG]) < u32(m_r[RTREG]);
					break;
				default:
					generate_exception(EXCEPTION_INVALIDOP);
					break;
				}
				break;
			case 0x01: // REGIMM
				/*
				 * Hardware testing has established that MIPS-1 processors do
				 * not decode bit 17 of REGIMM format instructions. This bit is
				 * used to add the "branch likely" instructions for MIPS-2 and
				 * later architectures.
				 *
				 * IRIX 5.3 inst(1M) uses this behaviour to distinguish MIPS-1
				 * from MIPS-2 processors; the latter nullify the delay slot
				 * instruction if the branch is not taken, whereas the former
				 * execute the delay slot instruction regardless.
				 */
				switch (m_multiply_to_gpr ? RTREG : (RTREG & 0x1d))
				{
				case 0x00: // BLTZ
					if (s32(m_r[RSREG]) < 0)
					{
						m_branch_state = BRANCH;
						m_branch_target = m_pc + 4 + (s32(SIMMVAL) << 2);
					}
					break;
				case 0x01: // BGEZ
					if (s32(m_r[RSREG]) >= 0)
					{
						m_branch_state = BRANCH;
						m_branch_target = m_pc + 4 + (s32(SIMMVAL) << 2);
					}
					break;
				case 0x10: // BLTZAL
					if (m_multiply_to_gpr)
						m_r[31] = m_pc + 8;
					if (s32(m_r[RSREG]) < 0)
					{
						m_branch_state = BRANCH;
						m_branch_target = m_pc + 4 + (s32(SIMMVAL) << 2);
						if (!m_multiply_to_gpr)
							m_r[31] = m_pc + 8;
					}
					break;
				case 0x11: // BGEZAL
					if (m_multiply_to_gpr)
						m_r[31] = m_pc + 8;
					if (s32(m_r[RSREG]) >= 0)
					{
						m_branch_state = BRANCH;
						m_branch_target = m_pc + 4 + (s32(SIMMVAL) << 2);
						if (!m_multiply_to_gpr)
							m_r[31] = m_pc + 8;
					}
					break;
				case 0x02: // R3900 BLTZL
					if (s32(m_r[RSREG]) < 0)
					{
						m_branch_state = BRANCH;
						m_branch_target = m_pc + 4 + (s32(SIMMVAL) << 2);
					}
					else
						m_branch_state = NULLIFY;
					break;
				case 0x03: // R3900 BGEZL
					if (s32(m_r[RSREG]) >= 0)
					{
						m_branch_state = BRANCH;
						m_branch_target = m_pc + 4 + (s32(SIMMVAL) << 2);
					}
					else
						m_branch_state = NULLIFY;
					break;
				case 0x12: // R3900 BLTZALL
					m_r[31] = m_pc + 8;
					if (s32(m_r[RSREG]) < 0)
					{
						m_branch_state = BRANCH;
						m_branch_target = m_pc + 4 + (s32(SIMMVAL) << 2);
					}
					else
						m_branch_state = NULLIFY;
					break;
				case 0x13: // R3900 BGEZALL
					m_r[31] = m_pc + 8;
					if (s32(m_r[RSREG]) >= 0)
					{
						m_branch_state = BRANCH;
						m_branch_target = m_pc + 4 + (s32(SIMMVAL) << 2);
					}
					else
						m_branch_state = NULLIFY;
					break;
				default:
					generate_exception(EXCEPTION_INVALIDOP);
					break;
				}
				break;
			case 0x02: // J
				m_branch_state = BRANCH;
				m_branch_target = ((m_pc + 4) & 0xf0000000) | (LIMMVAL << 2);
				break;
			case 0x03: // JAL
				m_branch_state = BRANCH;
				m_branch_target = ((m_pc + 4) & 0xf0000000) | (LIMMVAL << 2);
				m_r[31] = m_pc + 8;
				break;
			case 0x04: // BEQ
				if (m_r[RSREG] == m_r[RTREG])
				{
					m_branch_state = BRANCH;
					m_branch_target = m_pc + 4 + (s32(SIMMVAL) << 2);
				}
				break;
			case 0x05: // BNE
				if (m_r[RSREG] != m_r[RTREG])
				{
					m_branch_state = BRANCH;
					m_branch_target = m_pc + 4 + (s32(SIMMVAL) << 2);
				}
				break;
			case 0x06: // BLEZ
				if (s32(m_r[RSREG]) <= 0)
				{
					m_branch_state = BRANCH;
					m_branch_target = m_pc + 4 + (s32(SIMMVAL) << 2);
				}
				break;
			case 0x07: // BGTZ
				if (s32(m_r[RSREG]) > 0)
				{
					m_branch_state = BRANCH;
					m_branch_target = m_pc + 4 + (s32(SIMMVAL) << 2);
				}
				break;
			case 0x14: // R3900 BEQL
				if (!m_multiply_to_gpr)
					generate_exception(EXCEPTION_INVALIDOP);
				else if (m_r[RSREG] == m_r[RTREG])
				{
					m_branch_state = BRANCH;
					m_branch_target = m_pc + 4 + (s32(SIMMVAL) << 2);
				}
				else
					m_branch_state = NULLIFY;
				break;
			case 0x15: // R3900 BNEL
				if (!m_multiply_to_gpr)
					generate_exception(EXCEPTION_INVALIDOP);
				else if (m_r[RSREG] != m_r[RTREG])
				{
					m_branch_state = BRANCH;
					m_branch_target = m_pc + 4 + (s32(SIMMVAL) << 2);
				}
				else
					m_branch_state = NULLIFY;
				break;
			case 0x16: // R3900 BLEZL
				if (!m_multiply_to_gpr)
					generate_exception(EXCEPTION_INVALIDOP);
				else if (s32(m_r[RSREG]) <= 0)
				{
					m_branch_state = BRANCH;
					m_branch_target = m_pc + 4 + (s32(SIMMVAL) << 2);
				}
				else
					m_branch_state = NULLIFY;
				break;
			case 0x17: // R3900 BGTZL
				if (!m_multiply_to_gpr)
					generate_exception(EXCEPTION_INVALIDOP);
				else if (s32(m_r[RSREG]) > 0)
				{
					m_branch_state = BRANCH;
					m_branch_target = m_pc + 4 + (s32(SIMMVAL) << 2);
				}
				else
					m_branch_state = NULLIFY;
				break;
			case 0x08: // ADDI
				{
					u32 const sum = m_r[RSREG] + SIMMVAL;

					// overflow: (sign(addend0) == sign(addend1)) && (sign(addend0) != sign(sum))
					if (!BIT(m_r[RSREG] ^ s32(SIMMVAL), 31) && BIT(m_r[RSREG] ^ sum, 31))
						generate_exception(EXCEPTION_OVERFLOW);
					else
						m_r[RTREG] = sum;
				}
				break;
			case 0x09: // ADDIU
				m_r[RTREG] = m_r[RSREG] + SIMMVAL;
				break;
			case 0x0a: // SLTI
				m_r[RTREG] = s32(m_r[RSREG]) < s32(SIMMVAL);
				break;
			case 0x0b: // SLTIU
				m_r[RTREG] = u32(m_r[RSREG]) < u32(SIMMVAL);
				break;
			case 0x0c: // ANDI
				m_r[RTREG] = m_r[RSREG] & UIMMVAL;
				break;
			case 0x0d: // ORI
				m_r[RTREG] = m_r[RSREG] | UIMMVAL;
				break;
			case 0x0e: // XORI
				m_r[RTREG] = m_r[RSREG] ^ UIMMVAL;
				break;
			case 0x0f: // LUI
				m_r[RTREG] = UIMMVAL << 16;
				break;
			case 0x10: // COP0
				if (!(SR & SR_KUc) || (SR & SR_COP0))
					handle_cop0(op);
				else
					generate_exception(EXCEPTION_BADCOP0);
				break;
			case 0x11: // COP1
				handle_cop1(op);
				break;
			case 0x12: // COP2
				handle_cop2(op);
				break;
			case 0x13: // COP3
				handle_cop3(op);
				break;
			case 0x1c: // R3900 MADD/MADDU
				handle_special2(op);
				break;
			case 0x20: // LB
				load<u8>(SIMMVAL + m_r[RSREG], [this, op](s8 temp)
				{
					m_r[RTREG] = temp;
					set_gpr_delay(RTREG);
				});
				break;
			case 0x21: // LH
				load<u16>(SIMMVAL + m_r[RSREG], [this, op](s16 temp)
				{
					m_r[RTREG] = temp;
					set_gpr_delay(RTREG);
				});
				break;
			case 0x22: // LWL
				lwl(op);
				break;
			case 0x23: // LW
				load<u32>(SIMMVAL + m_r[RSREG], [this, op](u32 temp)
				{
					m_r[RTREG] = temp;
					set_gpr_delay(RTREG);
				});
				break;
			case 0x24: // LBU
				load<u8>(SIMMVAL + m_r[RSREG], [this, op](u8 temp)
				{
					m_r[RTREG] = temp;
					set_gpr_delay(RTREG);
				});
				break;
			case 0x25: // LHU
				load<u16>(SIMMVAL + m_r[RSREG], [this, op](u16 temp)
				{
					m_r[RTREG] = temp;
					set_gpr_delay(RTREG);
				});
				break;
			case 0x26: // LWR
				lwr(op);
				break;
			case 0x28: // SB
				store<u8>(SIMMVAL + m_r[RSREG], m_r[RTREG]);
				break;
			case 0x29: // SH
				store<u16>(SIMMVAL + m_r[RSREG], m_r[RTREG]);
				break;
			case 0x2a: // SWL
				swl(op);
				break;
			case 0x2b: // SW
				store<u32>(SIMMVAL + m_r[RSREG], m_r[RTREG]);
				break;
			case 0x2e: // SWR
				swr(op);
				break;
			case 0x2f: // CACHE
				handle_cache(op);
				break;
			case 0x31: // LWC1
				handle_cop1(op);
				break;
			case 0x32: // LWC2
				handle_cop2(op);
				break;
			case 0x33: // LWC3
				handle_cop3(op);
				break;
			case 0x39: // SWC1
				handle_cop1(op);
				break;
			case 0x3a: // SWC2
				handle_cop2(op);
				break;
			case 0x3b: // SWC3
				handle_cop3(op);
				break;
			default:
				generate_exception(EXCEPTION_INVALIDOP);
				break;
			}

			// clear register 0
			m_r[0] = 0;
			});
		}

		// update pc and branch state
		switch (m_branch_state)
		{
		case NONE:
			m_pc += 4;
			break;

		case DELAY:
			m_branch_state = NONE;
			m_pc = m_branch_target;
			break;

		case BRANCH:
			m_branch_state = DELAY;
			m_pc += 4;
			break;

		case NULLIFY:
			m_branch_state = NONE;
			m_pc += 8;
			break;

		case EXCEPTION:
			m_branch_state = NONE;
			break;
		}

		// DERET suppresses single-step at its return destination and, when
		// that instruction branches, through its delay slot as well.
		if (debug_step_suppressed)
			m_debug_step_suppress = m_branch_state == DELAY;

		// The R3900 divider runs beside the integer pipeline.  Account for
		// every elapsed core cycle, including another pipeline interlock, but
		// only the issue cycle when this instruction started a new divide.
		advance_divide(
				divide_started
						? 1U
						: 1U + unsigned(cycle_start - m_icount));
	}
}

void mips1core_device_base::execute_set_input(int irqline, int state)
{
	if (state != CLEAR_LINE)
		CAUSE |= CAUSE_IPEX0 << irqline;
	else
		CAUSE &= ~(CAUSE_IPEX0 << irqline);
}

device_memory_interface::space_config_vector mips1core_device_base::memory_space_config() const
{
	return space_config_vector {
		std::make_pair(AS_PROGRAM, (m_endianness == ENDIANNESS_BIG) ? &m_program_config_be : &m_program_config_le)
	};
}

bool mips1core_device_base::memory_translate(int spacenum, int intention, offs_t &address, address_space *&target_space)
{
	target_space = &space(spacenum);

	if (spacenum != AS_PROGRAM)
		return true;

	return translate(intention, address, true) != ERROR;
}

mips1core_device_base::translate_result mips1core_device_base::translate(int intention, offs_t &address, bool debug)
{
	// check for kernel memory address
	if (BIT(address, 31))
	{
		// check debug or kernel mode
		if (debug || !(SR & SR_KUc))
		{
			switch (address & 0xe0000000)
			{
			case 0x80000000: // kseg0: unmapped, cached, privileged
				address &= ~0xe0000000;
				return m_cache;

			case 0xa0000000: // kseg1: unmapped, uncached, privileged
				address &= ~0xe0000000;
				return UNCACHED;

			case 0xc0000000: // kseg2: mapped, cached, privileged
			case 0xe0000000:
				break;
			}
		}
		else if (SR & SR_KUc)
		{
			address_error(intention, address);

			return ERROR;
		}
	}
	else
		// kuseg physical addresses have a 1GB offset
		address += 0x40000000;

	return m_cache;
}

mips1core_device_base::translate_result r3900_device::translate(int intention, offs_t &address, bool debug)
{
	// The R3900 has no TLB.  Unlike the IDT-derived embedded cores above,
	// its kuseg addresses map directly to the corresponding physical address.
	translate_result result;
	if (!BIT(address, 31))
		result = m_cache;
	else
		result = mips1core_device_base::translate(intention, address, debug);

	// Disabled caches behave like uncached accesses: every access misses and
	// no refill takes place.  Instruction and data enables are independent.
	if (result == CACHED
			&& !BIT(m_cop0[COP0_Config], intention == TR_FETCH ? 5 : 4))
		return UNCACHED;

	return result;
}

void r3900_device::exception_enter()
{
	// DALc/IALc and DALp/IALp form the same three-level exception stack as
	// the Status register's current/previous/old mode bits.
	u32 const modes = m_cop0[COP0_Cache];
	m_cop0[COP0_Cache] =
			(modes & ~0x0000'3f00) | ((modes << 2) & 0x0000'3c00);
}

void r3900_device::handle_rfe()
{
	// R3900 RFE leaves the old Status and Cache modes intact while copying
	// old to previous and previous to current.
	SR = (SR & ~0x0000'000f) | ((SR >> 2) & 0x0000'000f);
	if (bool(SR & SR_KUc) ^ bool(SR & SR_KUp))
		debugger_privilege_hook();

	u32 const modes = m_cop0[COP0_Cache];
	m_cop0[COP0_Cache] =
			(modes & ~0x0000'0f00) | ((modes >> 2) & 0x0000'0f00);
}

u32 r3900_device::get_cop0_reg(unsigned const reg)
{
	switch (reg)
	{
	case COP0_Config:
	case COP0_Cache:
	case COP0_Debug:
	case COP0_DEPC:
		return m_cop0[reg];

	default:
		return mips1core_device_base::get_cop0_reg(reg);
	}
}

void r3900_device::set_cop0_reg(unsigned const reg, u32 const data)
{
	switch (reg)
	{
	case COP0_Config:
		// ICS/DCS are read-only implementation sizes.  Reserved bits read
		// zero, and setting Lock prevents all writes until reset.
		if (!BIT(m_cop0[COP0_Config], 7))
		{
			m_cop0[COP0_Config] =
					(m_cop0[COP0_Config] & 0x003f'0000)
					| (data & 0x0000'0fff);
			// RF selects the processor clock divided by 1, 2, 4, or 8.
			// External devices retain their independently configured clocks.
			set_clock_scale(
					1.0 / double(1U << BIT(m_cop0[COP0_Config], 10, 2)));
		}
		break;

	case COP0_Cache:
		// Only the six current/previous/old I/D auto-lock mode bits exist.
		m_cop0[COP0_Cache] = data & 0x0000'3f00;
		break;

	case COP0_Debug:
		// SSt and BsF are the only software-writable Debug fields.
		m_cop0[COP0_Debug] =
				(m_cop0[COP0_Debug] & ~(DEBUG_BSF | DEBUG_SST))
				| (data & (DEBUG_BSF | DEBUG_SST));
		break;

	case COP0_DEPC:
		m_cop0[COP0_DEPC] = data;
		break;

	default:
		mips1core_device_base::set_cop0_reg(reg, data);
		break;
	}
}

bool r3900_device::cache_auto_lock(bool icache) const
{
	// Debug mode forces cache auto-lock off.  The implemented TX39
	// configuration reserves instruction auto-lock.
	return !(m_cop0[COP0_Debug] & DEBUG_DM)
			&& !icache
			&& BIT(m_cop0[COP0_Cache], 8);
}

unsigned r3900_device::cache_refill_words(bool icache) const
{
	if (icache)
		return 4U << BIT(m_cop0[COP0_Config], 2, 2);

	// DCBR clear selects the data cache's native one-word line. Otherwise
	// DRSize encodes 4, 8, 16, or 32 words.
	return BIT(m_cop0[COP0_Config], 6)
			? 4U << BIT(m_cop0[COP0_Config], 0, 2)
			: 1U;
}

void r3900_device::invalidate_data_cache(u32 address, u32 bytes)
{
	if (!bytes)
		return;

	// Dino DMA uses physical addresses while the R3900 can cache the same
	// memory through its direct-mapped low-address segment.  Invalidate every
	// resident word touched by the external write so the next CPU load refills
	// it from memory.  Keep locked lines intact: DALc turns a way into on-chip
	// scratch storage rather than ordinary coherent memory.
	u64 const end = u64(address) + bytes;
	for (u64 current = address & ~u32(3); current < end; current += 4)
	{
		u32 const word = u32(current);
		unsigned const index = m_dcache.index(word);
		for (unsigned way = 0; way < m_dcache.ways; ++way)
		{
			struct cache::line &line = m_dcache.at(index, way);
			if (!line.locked
					&& !((line.tag ^ word)
							& (-m_dcache.way_size() | cache::line::INV)))
				line.invalidate();
		}
	}
}

bool mips1core_device_base::reads_gpr(u32 const op, unsigned const reg) const
{
	if (!reg)
		return false;

	unsigned const rs = BIT(op, 21, 5);
	unsigned const rt = BIT(op, 16, 5);
	auto const reads_rs = [reg, rs]() { return rs == reg; };
	auto const reads_rt = [reg, rt]() { return rt == reg; };

	switch (op >> 26)
	{
	case 0x00: // SPECIAL
		switch (op & 63)
		{
		case 0x00: // SLL
		case 0x02: // SRL
		case 0x03: // SRA
			return reads_rt();

		case 0x04: // SLLV
		case 0x06: // SRLV
		case 0x07: // SRAV
		case 0x18: // MULT
		case 0x19: // MULTU
		case 0x1a: // DIV
		case 0x1b: // DIVU
		case 0x20: // ADD
		case 0x21: // ADDU
		case 0x22: // SUB
		case 0x23: // SUBU
		case 0x24: // AND
		case 0x25: // OR
		case 0x26: // XOR
		case 0x27: // NOR
		case 0x2a: // SLT
		case 0x2b: // SLTU
			return reads_rs() || reads_rt();

		case 0x08: // JR
		case 0x09: // JALR
		case 0x11: // MTHI
		case 0x13: // MTLO
			return reads_rs();

		default:
			return false;
		}

	case 0x01: // REGIMM
	case 0x06: // BLEZ
	case 0x07: // BGTZ
	case 0x08: // ADDI
	case 0x09: // ADDIU
	case 0x0a: // SLTI
	case 0x0b: // SLTIU
	case 0x0c: // ANDI
	case 0x0d: // ORI
	case 0x0e: // XORI
		return reads_rs();

	case 0x04: // BEQ
	case 0x05: // BNE
	case 0x14: // R3900 BEQL
	case 0x15: // R3900 BNEL
	case 0x1c: // R3900 MADD/MADDU
		return reads_rs() || reads_rt();

	case 0x16: // R3900 BLEZL
	case 0x17: // R3900 BGTZL
		return reads_rs();

	case 0x10: // COP0
	case 0x11: // COP1
	case 0x12: // COP2
	case 0x13: // COP3
		// Move/control-to-coprocessor instructions source a GPR in rt.
		return (rs == 0x04 || rs == 0x06) && reads_rt();

	case 0x20: // LB
	case 0x21: // LH
	case 0x22: // LWL
	case 0x23: // LW
	case 0x24: // LBU
	case 0x25: // LHU
	case 0x26: // LWR
	case 0x2f: // CACHE
	case 0x31: // LWC1
	case 0x32: // LWC2
	case 0x33: // LWC3
	case 0x39: // SWC1
	case 0x3a: // SWC2
	case 0x3b: // SWC3
		return reads_rs();

	case 0x28: // SB
	case 0x29: // SH
	case 0x2a: // SWL
	case 0x2b: // SW
	case 0x2e: // SWR
		return reads_rs() || reads_rt();

	default:
		return false;
	}
}

void mips1core_device_base::gpr_interlock(u32 const op)
{
	if (m_gpr_delay && reads_gpr(op, m_gpr_delay))
		m_icount--;

	m_gpr_delay = 0;
}

void mips1core_device_base::set_gpr_delay(unsigned const reg)
{
	if (m_multiply_to_gpr)
		m_gpr_delay = reg;
}

void mips1core_device_base::cancel_divide()
{
	if (m_multiply_to_gpr)
		m_divide_cycles = 0;
}

void mips1core_device_base::divide_interlock()
{
	if (!m_divide_cycles)
		return;

	m_icount -= m_divide_cycles;
	m_hi = m_divide_hi;
	m_lo = m_divide_lo;
	m_divide_cycles = 0;
}

void mips1core_device_base::advance_divide(unsigned const cycles)
{
	if (!m_divide_cycles)
		return;

	if (cycles < m_divide_cycles)
	{
		m_divide_cycles -= cycles;
		return;
	}

	m_hi = m_divide_hi;
	m_lo = m_divide_lo;
	m_divide_cycles = 0;
}

bool r3900_device::cache_store_allocate() const
{
	// The R3900 data cache is write-through without write allocation.
	return false;
}

void r3900_device::handle_cache(u32 const op)
{
	offs_t address = m_r[RSREG] + SIMMVAL;

	switch (RTREG)
	{
	case 0x00: // instruction cache index invalidate
		if (!BIT(m_cop0[COP0_Config], 5))
		{
			// One instruction-cache tag covers four words.
			for (unsigned word = 0; word < 4; ++word)
				std::get<0>(
						cache_lookup(address + word * 4, false, true))
						.invalidate();
		}
		break;

	case 0x05: // data cache index LRU bit clear
		m_dcache.lru[m_dcache.index(address)] = 0;
		break;

	case 0x09: // data cache index lock bit clear
		for (unsigned way = 0; way < m_dcache.ways; ++way)
			m_dcache.at(m_dcache.index(address), way).locked = 0;
		break;

	case 0x11: // data cache hit invalidate
		if (translate(TR_READ, address, false) == CACHED)
		{
			auto [line, miss] = cache_lookup(address, false);
			if (!miss)
			{
				line.invalidate();
				if (!line.locked)
				{
					unsigned const index = m_dcache.index(address);
					for (unsigned way = 0; way < m_dcache.ways; ++way)
					{
						if (&m_dcache.at(index, way) == &line)
						{
							m_dcache.lru[index] = way;
							break;
						}
					}
				}
			}
		}
		break;

	default:
		generate_exception(EXCEPTION_INVALIDOP);
		break;
	}
}

void mips1core_device_base::handle_special2(u32 const op)
{
	generate_exception(EXCEPTION_INVALIDOP);
}

void r3900_device::handle_special2(u32 const op)
{
	if (BIT(op, 6, 5))
	{
		generate_exception(EXCEPTION_INVALIDOP);
		return;
	}

	u64 product;

	switch (op & 63)
	{
	case 0x00: // MADD
		product = mul_32x32(m_r[RSREG], m_r[RTREG]);
		break;

	case 0x01: // MADDU
		product = mulu_32x32(m_r[RSREG], m_r[RTREG]);
		break;

	default:
		generate_exception(EXCEPTION_INVALIDOP);
		return;
	}

	// MADD and MADDU read HI:LO, so they interlock only while an independent
	// divide is still producing that accumulator.
	divide_interlock();

	u64 const accumulator = (u64(m_hi) << 32) | m_lo;
	u64 const result = accumulator + product;
	m_lo = u32(result);
	m_hi = u32(result >> 32);
	m_r[RDREG] = m_lo;

	set_gpr_delay(RDREG);
}

std::unique_ptr<util::disasm_interface> mips1core_device_base::create_disassembler()
{
	return std::make_unique<mips1_disassembler>(m_multiply_to_gpr);
}

void mips1core_device_base::generate_debug_exception(u32 const cause)
{
	// Debug exceptions use their own state and vector.  Ordinary Status,
	// Cause and EPC state is deliberately left untouched.
	m_gpr_delay = 0;
	m_cop0[COP0_DEPC] = m_pc;

	u32 debug = m_cop0[COP0_Debug] & (DEBUG_BSF | DEBUG_SST);
	if (m_branch_state == DELAY)
	{
		m_cop0[COP0_DEPC] -= 4;
		debug |= DEBUG_DBD;
	}

	m_cop0[COP0_Debug] = debug | DEBUG_DM | cause;
	m_branch_state = EXCEPTION;
	m_pc = 0xbfc0'0200;
}

void mips1core_device_base::generate_exception(u32 exception, bool refill)
{
	// An exception flushes the integer pipeline.  A TX39 divide continues in
	// its independent unit, but a one-cycle GPR dependency does not carry into
	// the exception handler.
	m_gpr_delay = 0;

	// set the exception PC
	m_cop0[COP0_EPC] = m_pc;

	// load the cause register
	CAUSE = (CAUSE & CAUSE_IP) | exception;

	// if in a branch delay slot, restart the branch
	if (m_branch_state == DELAY)
	{
		m_cop0[COP0_EPC] -= 4;
		CAUSE |= CAUSE_BD;
	}
	m_branch_state = EXCEPTION;

	if (refill)
		m_pc = (SR & SR_BEV) ? 0xbfc00100 : 0x80000000;
	else
		m_pc = (SR & SR_BEV) ? 0xbfc00180 : 0x80000080;

	// hook exception in caller context enabling debugger access to memory parameters
	debugger_exception_hook(exception);

	exception_enter();

	// shift the exception bits
	SR = (SR & ~SR_KUIE) | ((SR << 2) & SR_KUIEop);

	if (SR & SR_KUp)
		debugger_privilege_hook();
}

void mips1core_device_base::exception_enter()
{
}

void mips1core_device_base::address_error(int intention, u32 const address)
{
	if (!machine().side_effects_disabled())
	{
		logerror("address_error 0x%08x (%s)\n", address, machine().describe_context());

		m_cop0[COP0_BadVAddr] = address;

		generate_exception((intention == TR_WRITE) ? EXCEPTION_ADDRSTORE : EXCEPTION_ADDRLOAD);

		// address errors shouldn't typically occur, so a breakpoint is handy
		machine().debug_break();
	}
}

void mips1core_device_base::handle_cop0(u32 const op)
{
	switch (RSREG)
	{
	case 0x00: // MFC0
		m_r[RTREG] = get_cop0_reg(RDREG);
		break;
	case 0x04: // MTC0
		set_cop0_reg(RDREG, m_r[RTREG]);
		break;
	case 0x08: // BC0
		handle_cop_branch(0, op);
		break;
	case 0x10: // COP0
		switch (op & 31)
		{
			case 0x10: // RFE
				handle_rfe();
				break;
			case 0x1f: // R3900 DERET
				if (m_multiply_to_gpr && (m_cop0[COP0_Debug] & DEBUG_DM))
				{
					m_pc = m_cop0[COP0_DEPC];
					m_branch_state = EXCEPTION;
					m_cop0[COP0_Debug] &= ~DEBUG_DM;
					bool const was_user = bool(SR & SR_KUc);
					SR |= SR_KUc | SR_IEc;
					if (!was_user)
						debugger_privilege_hook();
					m_debug_step_suppress = true;
				}
				else
					generate_exception(EXCEPTION_INVALIDOP);
				break;
			default:
				generate_exception(EXCEPTION_INVALIDOP);
				break;
		}
		break;
	default:
		generate_exception(EXCEPTION_INVALIDOP);
		break;
	}
}

void mips1core_device_base::handle_rfe()
{
	SR = (SR & ~SR_KUIE) | ((SR >> 2) & SR_KUIEpc);
	if (bool(SR & SR_KUc) ^ bool(SR & SR_KUp))
		debugger_privilege_hook();
}

u32 mips1core_device_base::get_cop0_reg(unsigned const reg)
{
	return m_cop0[reg];
}

void mips1core_device_base::set_cop0_reg(unsigned const reg, u32 const data)
{
	switch (reg)
	{
	case COP0_Status:
		{
			u32 const delta = SR ^ data;

			if ((delta & SR_IsC) && (m_cache == UNCACHED))
				fatalerror("mips1: cannot isolate non-existent cache (%s)\n", machine().describe_context());

			m_cop0[COP0_Status] = data;

			if ((delta & SR_KUc) && (m_branch_state != EXCEPTION))
				debugger_privilege_hook();
		}
		break;

	case COP0_Cause:
		CAUSE = (CAUSE & CAUSE_IPEX) | (data & ~CAUSE_IPEX);
		break;

	case COP0_PRId:
		// read-only register
		break;

	default:
		m_cop0[reg] = data;
		break;
	}
}

void mips1core_device_base::handle_cop1(u32 const op)
{
	if (!(SR & SR_COP1))
		generate_exception(EXCEPTION_BADCOP1);
	else if (RSREG == 0x08) // BC1
		handle_cop_branch(1, op);
	else
		generate_exception(EXCEPTION_INVALIDOP);
}

void mips1core_device_base::handle_cop_branch(unsigned const cop, u32 const op)
{
	if (RTREG > (m_multiply_to_gpr ? 0x03 : 0x01))
	{
		generate_exception(EXCEPTION_INVALIDOP);
		return;
	}

	bool const taken = bool(m_in_brcond[cop]()) == bool(RTREG & 1);
	if (taken)
	{
		m_branch_state = BRANCH;
		m_branch_target = m_pc + 4 + (s32(SIMMVAL) << 2);
	}
	else if (BIT(RTREG, 1)) // R3900 BCzFL/BCzTL
		m_branch_state = NULLIFY;
}

void mips1core_device_base::handle_cache(u32 const)
{
	generate_exception(EXCEPTION_INVALIDOP);
}

bool mips1core_device_base::cache_auto_lock(bool icache) const
{
	return false;
}

unsigned mips1core_device_base::cache_refill_words(bool icache) const
{
	return 1;
}

bool mips1core_device_base::cache_store_allocate() const
{
	return true;
}

void mips1core_device_base::handle_cop2(u32 const op)
{
	if (SR & SR_COP2)
	{
		switch (RSREG)
		{
		case 0x08: // BC2
			handle_cop_branch(2, op);
			break;
		default:
			generate_exception(EXCEPTION_INVALIDOP);
			break;
		}
	}
	else
		generate_exception(EXCEPTION_BADCOP2);
}

void mips1core_device_base::handle_cop3(u32 const op)
{
	if (SR & SR_COP3)
	{
		switch (RSREG)
		{
		case 0x08: // BC3
			handle_cop_branch(3, op);
			break;
		default:
			generate_exception(EXCEPTION_INVALIDOP);
			break;
		}
	}
	else
		generate_exception(EXCEPTION_BADCOP3);
}

void mips1core_device_base::lwl(u32 const op)
{
	offs_t const offset = SIMMVAL + m_r[RSREG];
	load<u32, false>(offset, [this, op, offset](u32 temp)
	{
		unsigned const shift = ((offset & 3) ^ (m_endianness == ENDIANNESS_LITTLE ? 3 : 0)) << 3;

		m_r[RTREG] = (m_r[RTREG] & ~u32(0xffffffffU << shift)) | (temp << shift);
		set_gpr_delay(RTREG);
	});
}

void mips1core_device_base::lwr(u32 const op)
{
	offs_t const offset = SIMMVAL + m_r[RSREG];
	load<u32, false>(offset, [this, op, offset](u32 temp)
	{
		unsigned const shift = ((offset & 3) ^ (m_endianness == ENDIANNESS_LITTLE ? 0 : 3)) << 3;

		m_r[RTREG] = (m_r[RTREG] & ~u32(0xffffffffU >> shift)) | (temp >> shift);
		set_gpr_delay(RTREG);
	});
}

void mips1core_device_base::swl(u32 const op)
{
	offs_t const offset = SIMMVAL + m_r[RSREG];
	unsigned const shift = ((offset & 3) ^ (m_endianness == ENDIANNESS_LITTLE ? 3 : 0)) << 3;

	store<u32, false>(offset, m_r[RTREG] >> shift, 0xffffffffU >> shift);
}

void mips1core_device_base::swr(u32 const op)
{
	offs_t const offset = SIMMVAL + m_r[RSREG];
	unsigned const shift = ((offset & 3) ^ (m_endianness == ENDIANNESS_LITTLE ? 0 : 3)) << 3;

	store<u32, false>(offset, m_r[RTREG] << shift, 0xffffffffU << shift);
}

/*
 * This function determines the active cache (instruction or data) depending on
 * the icache parameter and the status register SwC (swap caches) flag. A set
 * within the cache is selected based upon the low address bits, and every way
 * is compared with the upper address tag.
 *
 * A miss selects an invalid unlocked way before the LRU way. If the cache
 * lookup misses and the invalidate parameter evaluates to true, the selected
 * cache line tag is updated to match the input address and invalidated.
 *
 * The function returns the selected line and the miss state.
 *
 * Cache backing remains word-granular. Devices may refill several consecutive
 * words for one miss while retaining this per-word lookup and tag model.
 */
std::tuple<struct mips1core_device_base::cache::line &, bool> mips1core_device_base::cache_lookup(u32 address, bool invalidate, bool icache)
{
	// cache line data is word-addressed
	address &= ~3;

	// select instruction or data cache
	struct cache &c = (icache ^ bool(SR & SR_SwC)) ? m_icache : m_dcache;
	unsigned const index = c.index(address);

	// clear cache parity error
	SR &= ~SR_PE;

	// Compare every way before selecting a replacement.
	unsigned selected = c.lru[index];
	bool miss = true;
	for (unsigned way = 0; way < c.ways; ++way)
	{
		struct cache::line &candidate = c.at(index, way);
		if (!((candidate.tag ^ address)
					& (-c.way_size() | cache::line::INV)))
		{
			selected = way;
			miss = false;
			break;
		}
	}

	// Prefer an invalid, unlocked way. A locked index can replace only its
	// unlocked way, independent of the ordinary LRU selector.
	if (miss)
	{
		for (unsigned way = 0; way < c.ways; ++way)
		{
			if (!c.at(index, way).locked
					&& (c.at(index, way).tag & cache::line::INV))
			{
				selected = way;
				break;
			}
		}
		for (unsigned way = 0; c.ways > 1 && way < c.ways; ++way)
		{
			if (c.at(index, way).locked)
				selected = way ^ 1;
		}
	}

	struct cache::line &l = c.at(index, selected);

	// A miss is usually followed by line replacement.
	if (miss && invalidate)
	{
		l.tag = (address & -c.way_size()) | cache::line::INV;
		l.locked = 0;
	}

	if (c.ways > 1 && (!miss || invalidate))
	{
		// While one way is locked the replacement selector must continue to
		// name the other way. Otherwise the accessed way becomes most recent.
		bool locked = false;
		for (unsigned way = 0; way < c.ways; ++way)
		{
			if (c.at(index, way).locked)
			{
				c.lru[index] = way ^ 1;
				locked = true;
			}
		}
		if (!locked)
			c.lru[index] = selected ^ 1;
	}

	return std::tie(l, miss);
}

void mips1core_device_base::cache_lock(u32 address, bool icache)
{
	if (!cache_auto_lock(icache))
		return;

	address &= ~3;
	struct cache &c = (icache ^ bool(SR & SR_SwC)) ? m_icache : m_dcache;
	if (c.ways < 2)
		return;
	unsigned const index = c.index(address);
	for (unsigned way = 0; way < c.ways; ++way)
	{
		struct cache::line &candidate = c.at(index, way);
		if (!((candidate.tag ^ address)
					& (-c.way_size() | cache::line::INV)))
		{
			for (unsigned other = 0; other < c.ways; ++other)
				c.at(index, other).locked = other == way;
			if (c.ways > 1)
				c.lru[index] = way ^ 1;
			break;
		}
	}
}

bool mips1core_device_base::cache_refill(u32 address, bool icache)
{
	unsigned const words = cache_refill_words(icache);
	u32 const start = address & ~(words * 4 - 1);

	for (unsigned word = 0; word < words; ++word)
	{
		u32 const refill_address = start + word * 4;
		struct cache::line &line =
				std::get<0>(cache_lookup(refill_address, true, icache));
		u32 const data = space(AS_PROGRAM).read_dword(refill_address);
		if (m_bus_error)
		{
			m_bus_error = false;
			generate_exception(
					icache ? EXCEPTION_BUSINST : EXCEPTION_BUSDATA);
			return false;
		}

		line.update(data);
		cache_lock(refill_address, icache);
	}

	return true;
}

// compute bit position of sub-unit within a word given endianness and address
template <typename T>
unsigned mips1core_device_base::shift_factor(u32 address) const
{
	if constexpr (sizeof(T) == 1)
		return ((m_endianness == ENDIANNESS_BIG) ? (address & 3) ^ 3 : (address & 3)) * 8;
	else if constexpr (sizeof(T) == 2)
		return ((m_endianness == ENDIANNESS_BIG) ? (address & 2) ^ 2 : (address & 2)) * 8;
	else
		return 0;
}

template <typename T, bool Aligned, typename U>
std::enable_if_t<std::is_convertible<U, std::function<void(T)>>::value, void> mips1core_device_base::load(offs_t address, U &&apply)
{
	// alignment error
	if (Aligned && (address & (sizeof(T) - 1)))
	{
		address_error(TR_READ, address);
		return;
	}

	T data;
	if (!(SR & SR_IsC))
	{
		translate_result const t = translate(TR_READ, address, false);
		if (t == ERROR)
			return;

		// align address for ld[lr] instructions
		if (!Aligned)
			address &= ~(sizeof(T) - 1);

		if (t == CACHED)
		{
			auto [l, miss] = cache_lookup(address, true);

			if (miss)
			{
				if (!cache_refill(address, false))
					return;
			}

			data = l.data >> shift_factor<T>(address);
			cache_lock(address);
		}
		else
		{
			if constexpr (sizeof(T) == 4)
				data = space(AS_PROGRAM).read_dword(address);
			else if constexpr (sizeof(T) == 2)
				data = space(AS_PROGRAM).read_word(address);
			else if constexpr (sizeof(T) == 1)
				data = space(AS_PROGRAM).read_byte(address);

			if (m_bus_error)
			{
				m_bus_error = false;
				generate_exception(EXCEPTION_BUSDATA);

				return;
			}
		}
	}
	else
	{
		// when isolated, loads always hit the cache and the status register
		// CM flag reflects the actual hit/miss state
		auto [l, miss] = cache_lookup(address & ~0xe000'0000, false);

		if (miss)
			SR |= SR_CM;
		else
			SR &= ~SR_CM;

		data = l.data >> shift_factor<T>(address);
	}

	apply(data);
}

template <typename T, bool Aligned>
void mips1core_device_base::store(offs_t address, T data, T mem_mask)
{
	// alignment error
	if (Aligned && (address & (sizeof(T) - 1)))
	{
		address_error(TR_WRITE, address);
		return;
	}

	if (!(SR & SR_IsC))
	{
		translate_result const t = translate(TR_WRITE, address, false);
		if (t == ERROR)
			return;
		bool write_memory = true;

		// align address for sd[lr] instructions
		if (!Aligned)
			address &= ~(sizeof(T) - 1);

		if (t == CACHED)
		{
			auto [l, miss] = cache_lookup(
					address,
					sizeof(T) == 4 && cache_store_allocate());

			// Most MIPS-I caches allocate a full-word store miss. The R3900
			// data cache is explicitly write-through without write allocation.
			if constexpr (Aligned && sizeof(T) == 4)
			{
				if (!miss || cache_store_allocate())
				{
					l.update(data);
					cache_lock(address);
					write_memory = !l.locked;
				}
			}
			else if (!miss)
			{
				if (!m_cache_pws)
				{
					// reload the cache line from memory
					u32 const data = space(AS_PROGRAM).read_dword(address);
					if (m_bus_error)
					{
						m_bus_error = false;
						generate_exception(EXCEPTION_BUSDATA);

						return;
					}

					l.update(data);
				}

				// merge data into the cache
				unsigned const shift = shift_factor<T>(address);
				l.update(u32(data) << shift, u32(mem_mask) << shift);
				cache_lock(address);
				write_memory = !l.locked;
			}
		}

		// Uncached and ordinary cached stores reach memory. A TX39 locked-line
		// hit updates only the cache until software clears the lock and stores
		// the value again.
		if (write_memory)
		{
			if constexpr (sizeof(T) == 4)
				space(AS_PROGRAM).write_dword(address, T(data), mem_mask);
			else if constexpr (sizeof(T) == 2)
				space(AS_PROGRAM).write_word(address, T(data), mem_mask);
			else if constexpr (sizeof(T) == 1)
				space(AS_PROGRAM).write_byte(address, T(data));
		}
	}
	else
	{
		// when isolated, full word stores update the cache, while partial word
		// stores invalidate the cache line
		auto [l, miss] = cache_lookup(address & ~0xe000'0000, true);

		if constexpr (Aligned && sizeof(T) == 4)
			l.update(data, mem_mask);
		else
			l.invalidate();
	}
}

void mips1core_device_base::fetch(offs_t address, std::function<void(u32)> &&apply)
{
	// alignment error
	if (address & 3)
		address_error(TR_FETCH, address);

	translate_result const t = translate(TR_FETCH, address, false);
	if (t == ERROR)
		return;

	u32 data;
	if (t == CACHED)
	{
		auto [l, miss] = cache_lookup(address, true, true);

		if (miss)
		{
			if (!cache_refill(address, true))
				return;
		}

		data = l.data;
	}
	else
	{
		data = space(AS_PROGRAM).read_dword(address);

		if (m_bus_error)
		{
			m_bus_error = false;
			generate_exception(EXCEPTION_BUSINST);

			return;
		}
	}

	apply(data);
}

void mips1_device_base::device_start()
{
	mips1core_device_base::device_start();

	// cop0 tlb registers
	state_add(MIPS1_COP0 + COP0_Index, "Index", m_cop0[COP0_Index]);
	state_add(MIPS1_COP0 + COP0_Random, "Random", m_cop0[COP0_Random]);
	state_add(MIPS1_COP0 + COP0_EntryLo, "EntryLo", m_cop0[COP0_EntryLo]);
	state_add(MIPS1_COP0 + COP0_EntryHi, "EntryHi", m_cop0[COP0_EntryHi]);
	state_add(MIPS1_COP0 + COP0_Context, "Context", m_cop0[COP0_Context]);

	// cop1 registers
	if (m_fcr0)
	{
		state_add(MIPS1_FCR31, "FCSR", m_fcr31);
		for (unsigned i = 0; i < std::size(m_f); i++)
			state_add(MIPS1_F0 + i, util::string_format("F%d", i * 2).c_str(), m_f[i]);
	}

	save_item(NAME(m_reset_time));
	save_item(NAME(m_tlb));

	save_item(NAME(m_fcr30));
	save_item(NAME(m_fcr31));
	save_item(NAME(m_f));
}

void mips1_device_base::device_reset()
{
	mips1core_device_base::device_reset();

	// tlb is not shut down
	m_cop0[COP0_Status] &= ~SR_TS;

	m_reset_time = total_cycles();

	// initialize tlb mru index with identity mapping
	for (unsigned i = 0; i < std::size(m_tlb); i++)
	{
		m_tlb_mru[TR_READ][i] = i;
		m_tlb_mru[TR_WRITE][i] = i;
		m_tlb_mru[TR_FETCH][i] = i;
	}
}

void mips1_device_base::handle_cop0(u32 const op)
{
	switch (op)
	{
	case 0x42000001: // TLBR - read tlb
		{
			u8 const index = (m_cop0[COP0_Index] >> 8) & 0x3f;

			m_cop0[COP0_EntryHi] = m_tlb[index][0];
			m_cop0[COP0_EntryLo] = m_tlb[index][1];
		}
		break;

	case 0x42000002: // TLBWI - write tlb (indexed)
		{
			u8 const index = (m_cop0[COP0_Index] >> 8) & 0x3f;

			m_tlb[index][0] = m_cop0[COP0_EntryHi];
			m_tlb[index][1] = m_cop0[COP0_EntryLo];

			LOGMASKED(LOG_TLB, "asid %2d tlb write index %2d vpn 0x%08x pfn 0x%08x %c%c%c%c (%s)\n",
				(m_cop0[COP0_EntryHi] & EH_ASID) >> 6, index, m_cop0[COP0_EntryHi] & EH_VPN, m_cop0[COP0_EntryLo] & EL_PFN,
				m_cop0[COP0_EntryLo] & EL_N ? 'N' : '-',
				m_cop0[COP0_EntryLo] & EL_D ? 'D' : '-',
				m_cop0[COP0_EntryLo] & EL_V ? 'V' : '-',
				m_cop0[COP0_EntryLo] & EL_G ? 'G' : '-',
				machine().describe_context());
		}
		break;

	case 0x42000006: // TLBWR - write tlb (random)
		{
			u8 const random = get_cop0_reg(COP0_Random) >> 8;

			m_tlb[random][0] = m_cop0[COP0_EntryHi];
			m_tlb[random][1] = m_cop0[COP0_EntryLo];

			LOGMASKED(LOG_TLB, "asid %2d tlb write random %2d vpn 0x%08x pfn 0x%08x %c%c%c%c (%s)\n",
				(m_cop0[COP0_EntryHi] & EH_ASID) >> 6, random, m_cop0[COP0_EntryHi] & EH_VPN, m_cop0[COP0_EntryLo] & EL_PFN,
				m_cop0[COP0_EntryLo] & EL_N ? 'N' : '-',
				m_cop0[COP0_EntryLo] & EL_D ? 'D' : '-',
				m_cop0[COP0_EntryLo] & EL_V ? 'V' : '-',
				m_cop0[COP0_EntryLo] & EL_G ? 'G' : '-',
				machine().describe_context());
		}
		break;

	case 0x42000008: // TLBP - probe tlb
		m_cop0[COP0_Index] = 0x80000000;
		for (u8 index = 0; index < 64; index++)
		{
			// test vpn and optionally asid
			u32 const mask = (m_tlb[index][1] & EL_G) ? EH_VPN : EH_VPN | EH_ASID;
			if ((m_tlb[index][0] & mask) == (m_cop0[COP0_EntryHi] & mask))
			{
				LOGMASKED(LOG_TLB, "asid %2d tlb probe index %2d vpn 0x%08x (%s)\n",
					(m_cop0[COP0_EntryHi] & EH_ASID) >> 6, index, m_cop0[COP0_EntryHi] & mask, machine().describe_context());

				m_cop0[COP0_Index] = index << 8;
				break;
			}
		}
		if ((VERBOSE & LOG_TLB) && BIT(m_cop0[COP0_Index], 31))
			LOGMASKED(LOG_TLB, "asid %2d tlb probe miss vpn 0x%08x(%s)\n",
				(m_cop0[COP0_EntryHi] & EH_ASID) >> 6, m_cop0[COP0_EntryHi] & EH_VPN, machine().describe_context());
		break;

	default:
		mips1core_device_base::handle_cop0(op);
	}
}

u32 mips1_device_base::get_cop0_reg(unsigned const reg)
{
	// assume 64-entry tlb with 8 wired entries
	if (reg == COP0_Random)
		m_cop0[reg] = (63 - ((total_cycles() - m_reset_time) % 56)) << 8;

	return m_cop0[reg];
}

void mips1_device_base::set_cop0_reg(unsigned const reg, u32 const data)
{
	switch (reg)
	{
	case COP0_EntryHi:
		m_cop0[COP0_EntryHi] = data & EH_WM;
		break;

	case COP0_EntryLo:
		m_cop0[COP0_EntryLo] = data & EL_WM;
		break;

	case COP0_Context:
		m_cop0[COP0_Context] = (m_cop0[COP0_Context] & ~PTE_BASE) | (data & PTE_BASE);
		break;

	default:
		mips1core_device_base::set_cop0_reg(reg, data);
		break;
	}
}

void mips1_device_base::handle_cop1(u32 const op)
{
	if (!(SR & SR_COP1))
	{
		generate_exception(EXCEPTION_BADCOP1);
		return;
	}

	if (!m_fcr0)
		return;

	softfloat_exceptionFlags = 0;

	switch (op >> 26)
	{
	case 0x11: // COP1
		switch ((op >> 21) & 0x1f)
		{
		case 0x00: // MFC1
			if (FSREG & 1)
				// move the high half of the floating point register
				m_r[RTREG] = m_f[FSREG >> 1] >> 32;
			else
				// move the low half of the floating point register
				m_r[RTREG] = m_f[FSREG >> 1] >> 0;
			break;
		case 0x02: // CFC1
			switch (FSREG)
			{
			case 0:  m_r[RTREG] = m_fcr0; break;
			case 30: m_r[RTREG] = m_fcr30; break;
			case 31: m_r[RTREG] = m_fcr31; break;
				break;

			default:
				logerror("cfc1 undefined fpu control register %d (%s)\n", FSREG, machine().describe_context());
				break;
			}
			break;
		case 0x04: // MTC1
			if (FSREG & 1)
				// load the high half of the floating point register
				m_f[FSREG >> 1] = (u64(m_r[RTREG]) << 32) | u32(m_f[FSREG >> 1]);
			else
				// load the low half of the floating point register
				m_f[FSREG >> 1] = (m_f[FSREG >> 1] & ~0xffffffffULL) | m_r[RTREG];
			break;
		case 0x06: // CTC1
			switch (RDREG)
			{
			case 0: // register is read-only
				break;

			case 30:
				m_fcr30 = m_r[RTREG];
				break;

			case 31:
				m_fcr31 = m_r[RTREG];

				// update rounding mode
				switch (m_fcr31 & FCR31_RM)
				{
				case 0: softfloat_roundingMode = softfloat_round_near_even; break;
				case 1: softfloat_roundingMode = softfloat_round_minMag; break;
				case 2: softfloat_roundingMode = softfloat_round_max; break;
				case 3: softfloat_roundingMode = softfloat_round_min; break;
				}

				// exception check
				{
					bool const exception = (m_fcr31 & FCR31_CE) || (((m_fcr31 & FCR31_CM) >> 5) & (m_fcr31 & FCR31_EM));
					execute_set_input(m_fpu_irq, exception ? ASSERT_LINE : CLEAR_LINE);
				}
				break;

			default:
				logerror("ctc1 undefined fpu control register %d (%s)\n", RDREG, machine().describe_context());
				break;
			}
			break;
		case 0x08: // BC
			switch ((op >> 16) & 0x1f)
			{
			case 0x00: // BC1F
				if (!(m_fcr31 & FCR31_C))
				{
					m_branch_state = BRANCH;
					m_branch_target = m_pc + 4 + (s32(SIMMVAL) << 2);
				}
				break;
			case 0x01: // BC1T
				if (m_fcr31 & FCR31_C)
				{
					m_branch_state = BRANCH;
					m_branch_target = m_pc + 4 + (s32(SIMMVAL) << 2);
				}
				break;

			default:
				// unimplemented operation
				m_fcr31 |= FCR31_CE;
				execute_set_input(m_fpu_irq, ASSERT_LINE);
				break;
			}
			break;
		case 0x10: // S
			switch (op & 0x3f)
			{
			case 0x00: // ADD.S
				set_cop1_reg(FDREG >> 1, f32_add(float32_t{ u32(m_f[FSREG >> 1]) }, float32_t{ u32(m_f[FTREG >> 1]) }).v);
				break;
			case 0x01: // SUB.S
				set_cop1_reg(FDREG >> 1, f32_sub(float32_t{ u32(m_f[FSREG >> 1]) }, float32_t{ u32(m_f[FTREG >> 1]) }).v);
				break;
			case 0x02: // MUL.S
				set_cop1_reg(FDREG >> 1, f32_mul(float32_t{ u32(m_f[FSREG >> 1]) }, float32_t{ u32(m_f[FTREG >> 1]) }).v);
				break;
			case 0x03: // DIV.S
				set_cop1_reg(FDREG >> 1, f32_div(float32_t{ u32(m_f[FSREG >> 1]) }, float32_t{ u32(m_f[FTREG >> 1]) }).v);
				break;
			case 0x05: // ABS.S
				if (f32_lt(float32_t{ u32(m_f[FSREG >> 1]) }, float32_t{ 0 }))
					set_cop1_reg(FDREG >> 1, f32_mul(float32_t{ u32(m_f[FSREG >> 1]) }, i32_to_f32(-1)).v);
				else
					set_cop1_reg(FDREG >> 1, u32(m_f[FSREG >> 1]));
				break;
			case 0x06: // MOV.S
				if (FDREG & 1)
					if (FSREG & 1)
						// move high half to high half
						m_f[FDREG >> 1] = (m_f[FSREG >> 1] & ~0xffffffffULL) | u32(m_f[FDREG >> 1]);
					else
						// move low half to high half
						m_f[FDREG >> 1] = (m_f[FSREG >> 1] << 32) | u32(m_f[FDREG >> 1]);
				else
					if (FSREG & 1)
						// move high half to low half
						m_f[FDREG >> 1] = (m_f[FDREG >> 1] & ~0xffffffffULL) | (m_f[FSREG >> 1] >> 32);
					else
						// move low half to low half
						m_f[FDREG >> 1] = (m_f[FDREG >> 1] & ~0xffffffffULL) | u32(m_f[FSREG >> 1]);
				break;
			case 0x07: // NEG.S
				set_cop1_reg(FDREG >> 1, f32_mul(float32_t{ u32(m_f[FSREG >> 1]) }, i32_to_f32(-1)).v);
				break;

			case 0x21: // CVT.D.S
				set_cop1_reg(FDREG >> 1, f32_to_f64(float32_t{ u32(m_f[FSREG >> 1]) }).v);
				break;
			case 0x24: // CVT.W.S
				if (BIT(m_f[FSREG >> 1], 23, 8) == 0xff)
				{
					// +/- infinity or NaN
					m_fcr31 &= ~FCR31_CM;
					m_fcr31 |= FCR31_CE;
					execute_set_input(m_fpu_irq, ASSERT_LINE);
				}
				else
					set_cop1_reg(FDREG >> 1, f32_to_i32(float32_t{ u32(m_f[FSREG >> 1]) }, softfloat_roundingMode, true));
				break;

			case 0x30: // C.F.S (false)
				m_fcr31 &= ~FCR31_C;
				break;
			case 0x31: // C.UN.S (unordered)
				f32_eq(float32_t{ u32(m_f[FSREG >> 1]) }, float32_t{ u32(m_f[FTREG >> 1]) });
				if (softfloat_exceptionFlags & softfloat_flag_invalid)
					m_fcr31 |= FCR31_C;
				else
					m_fcr31 &= ~FCR31_C;
				break;
			case 0x32: // C.EQ.S (equal)
				if (f32_eq(float32_t{ u32(m_f[FSREG >> 1]) }, float32_t{ u32(m_f[FTREG >> 1]) }))
					m_fcr31 |= FCR31_C;
				else
					m_fcr31 &= ~FCR31_C;
				break;
			case 0x33: // C.UEQ.S (unordered equal)
				if (f32_eq(float32_t{ u32(m_f[FSREG >> 1]) }, float32_t{ u32(m_f[FTREG >> 1]) }) || (softfloat_exceptionFlags & softfloat_flag_invalid))
					m_fcr31 |= FCR31_C;
				else
					m_fcr31 &= ~FCR31_C;
				break;
			case 0x34: // C.OLT.S (less than)
				if (f32_lt(float32_t{ u32(m_f[FSREG >> 1]) }, float32_t{ u32(m_f[FTREG >> 1]) }))
					m_fcr31 |= FCR31_C;
				else
					m_fcr31 &= ~FCR31_C;
				break;
			case 0x35: // C.ULT.S (unordered less than)
				if (f32_lt(float32_t{ u32(m_f[FSREG >> 1]) }, float32_t{ u32(m_f[FTREG >> 1]) }) || (softfloat_exceptionFlags & softfloat_flag_invalid))
					m_fcr31 |= FCR31_C;
				else
					m_fcr31 &= ~FCR31_C;
				break;
			case 0x36: // C.OLE.S (less than or equal)
				if (f32_le(float32_t{ u32(m_f[FSREG >> 1]) }, float32_t{ u32(m_f[FTREG >> 1]) }))
					m_fcr31 |= FCR31_C;
				else
					m_fcr31 &= ~FCR31_C;
				break;
			case 0x37: // C.ULE.S (unordered less than or equal)
				if (f32_le(float32_t{ u32(m_f[FSREG >> 1]) }, float32_t{ u32(m_f[FTREG >> 1]) }) || (softfloat_exceptionFlags & softfloat_flag_invalid))
					m_fcr31 |= FCR31_C;
				else
					m_fcr31 &= ~FCR31_C;
				break;

			case 0x38: // C.SF.S (signalling false)
				f32_eq(float32_t{ u32(m_f[FSREG >> 1]) }, float32_t{ u32(m_f[FTREG >> 1]) });

				m_fcr31 &= ~FCR31_C;

				if (softfloat_exceptionFlags & softfloat_flag_invalid)
				{
					m_fcr31 |= FCR31_CV;
					execute_set_input(m_fpu_irq, ASSERT_LINE);
				}
				break;
			case 0x39: // C.NGLE.S (not greater, less than or equal)
				f32_eq(float32_t{ u32(m_f[FSREG >> 1]) }, float32_t{ u32(m_f[FTREG >> 1]) });

				if (softfloat_exceptionFlags & softfloat_flag_invalid)
				{
					m_fcr31 |= FCR31_C | FCR31_CV;
					execute_set_input(m_fpu_irq, ASSERT_LINE);
				}
				else
					m_fcr31 &= ~FCR31_C;
				break;
			case 0x3a: // C.SEQ.S (signalling equal)
				if (f32_eq(float32_t{ u32(m_f[FSREG >> 1]) }, float32_t{ u32(m_f[FTREG >> 1]) }))
					m_fcr31 |= FCR31_C;
				else
					m_fcr31 &= ~FCR31_C;

				if (softfloat_exceptionFlags & softfloat_flag_invalid)
				{
					m_fcr31 |= FCR31_CV;
					execute_set_input(m_fpu_irq, ASSERT_LINE);
				}
				break;
			case 0x3b: // C.NGL.S (not greater or less than)
				if (f32_eq(float32_t{ u32(m_f[FSREG >> 1]) }, float32_t{ u32(m_f[FTREG >> 1]) }) || (softfloat_exceptionFlags & softfloat_flag_invalid))
					m_fcr31 |= FCR31_C;
				else
					m_fcr31 &= ~FCR31_C;

				if (softfloat_exceptionFlags & softfloat_flag_invalid)
				{
					m_fcr31 |= FCR31_CV;
					execute_set_input(m_fpu_irq, ASSERT_LINE);
				}
				break;
			case 0x3c: // C.LT.S (less than)
				if (f32_lt(float32_t{ u32(m_f[FSREG >> 1]) }, float32_t{ u32(m_f[FTREG >> 1]) }))
					m_fcr31 |= FCR31_C;
				else
					m_fcr31 &= ~FCR31_C;

				if (softfloat_exceptionFlags & softfloat_flag_invalid)
				{
					m_fcr31 |= FCR31_CV;
					execute_set_input(m_fpu_irq, ASSERT_LINE);
				}
				break;
			case 0x3d: // C.NGE.S (not greater or equal)
				if (f32_lt(float32_t{ u32(m_f[FSREG >> 1]) }, float32_t{ u32(m_f[FTREG >> 1]) }) || (softfloat_exceptionFlags & softfloat_flag_invalid))
					m_fcr31 |= FCR31_C;
				else
					m_fcr31 &= ~FCR31_C;

				if (softfloat_exceptionFlags & softfloat_flag_invalid)
				{
					m_fcr31 |= FCR31_CV;
					execute_set_input(m_fpu_irq, ASSERT_LINE);
				}
				break;
			case 0x3e: // C.LE.S (less than or equal)
				if (f32_le(float32_t{ u32(m_f[FSREG >> 1]) }, float32_t{ u32(m_f[FTREG >> 1]) }))
					m_fcr31 |= FCR31_C;
				else
					m_fcr31 &= ~FCR31_C;

				if (softfloat_exceptionFlags & softfloat_flag_invalid)
				{
					m_fcr31 |= FCR31_CV;
					execute_set_input(m_fpu_irq, ASSERT_LINE);
				}
				break;
			case 0x3f: // C.NGT.S (not greater than)
				if (f32_le(float32_t{ u32(m_f[FSREG >> 1]) }, float32_t{ u32(m_f[FTREG >> 1]) }) || (softfloat_exceptionFlags & softfloat_flag_invalid))
					m_fcr31 |= FCR31_C;
				else
					m_fcr31 &= ~FCR31_C;

				if (softfloat_exceptionFlags & softfloat_flag_invalid)
				{
					m_fcr31 |= FCR31_CV;
					execute_set_input(m_fpu_irq, ASSERT_LINE);
				}
				break;

			default: // unimplemented operation
				m_fcr31 |= FCR31_CE;
				execute_set_input(m_fpu_irq, ASSERT_LINE);
				break;
			}
			break;
		case 0x11: // D
			switch (op & 0x3f)
			{
			case 0x00: // ADD.D
				set_cop1_reg(FDREG >> 1, f64_add(float64_t{ m_f[FSREG >> 1] }, float64_t{ m_f[FTREG >> 1] }).v);
				break;
			case 0x01: // SUB.D
				set_cop1_reg(FDREG >> 1, f64_sub(float64_t{ m_f[FSREG >> 1] }, float64_t{ m_f[FTREG >> 1] }).v);
				break;
			case 0x02: // MUL.D
				set_cop1_reg(FDREG >> 1, f64_mul(float64_t{ m_f[FSREG >> 1] }, float64_t{ m_f[FTREG >> 1] }).v);
				break;
			case 0x03: // DIV.D
				set_cop1_reg(FDREG >> 1, f64_div(float64_t{ m_f[FSREG >> 1] }, float64_t{ m_f[FTREG >> 1] }).v);
				break;

			case 0x05: // ABS.D
				if (f64_lt(float64_t{ m_f[FSREG >> 1] }, float64_t{ 0 }))
					set_cop1_reg(FDREG >> 1, f64_mul(float64_t{ m_f[FSREG >> 1] }, i32_to_f64(-1)).v);
				else
					set_cop1_reg(FDREG >> 1, m_f[FSREG >> 1]);
				break;
			case 0x06: // MOV.D
				m_f[FDREG >> 1] = m_f[FSREG >> 1];
				break;
			case 0x07: // NEG.D
				set_cop1_reg(FDREG >> 1, f64_mul(float64_t{ m_f[FSREG >> 1] }, i32_to_f64(-1)).v);
				break;

			case 0x20: // CVT.S.D
				set_cop1_reg(FDREG >> 1, f64_to_f32(float64_t{ m_f[FSREG >> 1] }).v);
				break;
			case 0x24: // CVT.W.D
				if (BIT(m_f[FSREG >> 1], 52, 11) == 0x7ff)
				{
					// +/- infinity or NaN
					m_fcr31 &= ~FCR31_CM;
					m_fcr31 |= FCR31_CE;
					execute_set_input(m_fpu_irq, ASSERT_LINE);
				}
				else
					set_cop1_reg(FDREG >> 1, f64_to_i32(float64_t{ m_f[FSREG >> 1] }, softfloat_roundingMode, true));
				break;

			case 0x30: // C.F.D (false)
				m_fcr31 &= ~FCR31_C;
				break;
			case 0x31: // C.UN.D (unordered)
				f64_eq(float64_t{ m_f[FSREG >> 1] }, float64_t{ m_f[FTREG >> 1] });
				if (softfloat_exceptionFlags & softfloat_flag_invalid)
					m_fcr31 |= FCR31_C;
				else
					m_fcr31 &= ~FCR31_C;
				break;
			case 0x32: // C.EQ.D (equal)
				if (f64_eq(float64_t{ m_f[FSREG >> 1] }, float64_t{ m_f[FTREG >> 1] }))
					m_fcr31 |= FCR31_C;
				else
					m_fcr31 &= ~FCR31_C;
				break;
			case 0x33: // C.UEQ.D (unordered equal)
				if (f64_eq(float64_t{ m_f[FSREG >> 1] }, float64_t{ m_f[FTREG >> 1] }) || (softfloat_exceptionFlags & softfloat_flag_invalid))
					m_fcr31 |= FCR31_C;
				else
					m_fcr31 &= ~FCR31_C;
				break;
			case 0x34: // C.OLT.D (less than)
				if (f64_lt(float64_t{ m_f[FSREG >> 1] }, float64_t{ m_f[FTREG >> 1] }))
					m_fcr31 |= FCR31_C;
				else
					m_fcr31 &= ~FCR31_C;
				break;
			case 0x35: // C.ULT.D (unordered less than)
				if (f64_lt(float64_t{ m_f[FSREG >> 1] }, float64_t{ m_f[FTREG >> 1] }) || (softfloat_exceptionFlags & softfloat_flag_invalid))
					m_fcr31 |= FCR31_C;
				else
					m_fcr31 &= ~FCR31_C;
				break;
			case 0x36: // C.OLE.D (less than or equal)
				if (f64_le(float64_t{ m_f[FSREG >> 1] }, float64_t{ m_f[FTREG >> 1] }))
					m_fcr31 |= FCR31_C;
				else
					m_fcr31 &= ~FCR31_C;
				break;
			case 0x37: // C.ULE.D (unordered less than or equal)
				if (f64_le(float64_t{ m_f[FSREG >> 1] }, float64_t{ m_f[FTREG >> 1] }) || (softfloat_exceptionFlags & softfloat_flag_invalid))
					m_fcr31 |= FCR31_C;
				else
					m_fcr31 &= ~FCR31_C;
				break;

			case 0x38: // C.SF.D (signalling false)
				f64_eq(float64_t{ m_f[FSREG >> 1] }, float64_t{ m_f[FTREG >> 1] });

				m_fcr31 &= ~FCR31_C;

				if (softfloat_exceptionFlags & softfloat_flag_invalid)
				{
					m_fcr31 |= FCR31_CV;
					execute_set_input(m_fpu_irq, ASSERT_LINE);
				}
				break;
			case 0x39: // C.NGLE.D (not greater, less than or equal)
				f64_eq(float64_t{ m_f[FSREG >> 1] }, float64_t{ m_f[FTREG >> 1] });

				if (softfloat_exceptionFlags & softfloat_flag_invalid)
				{
					m_fcr31 |= FCR31_C | FCR31_CV;
					execute_set_input(m_fpu_irq, ASSERT_LINE);
				}
				else
					m_fcr31 &= ~FCR31_C;
				break;
			case 0x3a: // C.SEQ.D (signalling equal)
				if (f64_eq(float64_t{ m_f[FSREG >> 1] }, float64_t{ m_f[FTREG >> 1] }))
					m_fcr31 |= FCR31_C;
				else
					m_fcr31 &= ~FCR31_C;

				if (softfloat_exceptionFlags & softfloat_flag_invalid)
				{
					m_fcr31 |= FCR31_CV;
					execute_set_input(m_fpu_irq, ASSERT_LINE);
				}
				break;
			case 0x3b: // C.NGL.D (not greater or less than)
				if (f64_eq(float64_t{ m_f[FSREG >> 1] }, float64_t{ m_f[FTREG >> 1] }) || (softfloat_exceptionFlags & softfloat_flag_invalid))
					m_fcr31 |= FCR31_C;
				else
					m_fcr31 &= ~FCR31_C;

				if (softfloat_exceptionFlags & softfloat_flag_invalid)
				{
					m_fcr31 |= FCR31_CV;
					execute_set_input(m_fpu_irq, ASSERT_LINE);
				}
				break;
			case 0x3c: // C.LT.D (less than)
				if (f64_lt(float64_t{ m_f[FSREG >> 1] }, float64_t{ m_f[FTREG >> 1] }))
					m_fcr31 |= FCR31_C;
				else
					m_fcr31 &= ~FCR31_C;

				if (softfloat_exceptionFlags & softfloat_flag_invalid)
				{
					m_fcr31 |= FCR31_CV;
					execute_set_input(m_fpu_irq, ASSERT_LINE);
				}
				break;
			case 0x3d: // C.NGE.D (not greater or equal)
				if (f64_lt(float64_t{ m_f[FSREG >> 1] }, float64_t{ m_f[FTREG >> 1] }) || (softfloat_exceptionFlags & softfloat_flag_invalid))
					m_fcr31 |= FCR31_C;
				else
					m_fcr31 &= ~FCR31_C;

				if (softfloat_exceptionFlags & softfloat_flag_invalid)
				{
					m_fcr31 |= FCR31_CV;
					execute_set_input(m_fpu_irq, ASSERT_LINE);
				}
				break;
			case 0x3e: // C.LE.D (less than or equal)
				if (f64_le(float64_t{ m_f[FSREG >> 1] }, float64_t{ m_f[FTREG >> 1] }))
					m_fcr31 |= FCR31_C;
				else
					m_fcr31 &= ~FCR31_C;

				if (softfloat_exceptionFlags & softfloat_flag_invalid)
				{
					m_fcr31 |= FCR31_CV;
					execute_set_input(m_fpu_irq, ASSERT_LINE);
				}
				break;
			case 0x3f: // C.NGT.D (not greater than)
				if (f64_le(float64_t{ m_f[FSREG >> 1] }, float64_t{ m_f[FTREG >> 1] }) || (softfloat_exceptionFlags & softfloat_flag_invalid))
					m_fcr31 |= FCR31_C;
				else
					m_fcr31 &= ~FCR31_C;

				if (softfloat_exceptionFlags & softfloat_flag_invalid)
				{
					m_fcr31 |= FCR31_CV;
					execute_set_input(m_fpu_irq, ASSERT_LINE);
				}
				break;

			default: // unimplemented operation
				m_fcr31 |= FCR31_CE;
				execute_set_input(m_fpu_irq, ASSERT_LINE);
				break;
			}
			break;
		case 0x14: // W
			switch (op & 0x3f)
			{
			case 0x20: // CVT.S.W
				set_cop1_reg(FDREG >> 1, i32_to_f32(s32(m_f[FSREG >> 1])).v);
				break;
			case 0x21: // CVT.D.W
				set_cop1_reg(FDREG >> 1, i32_to_f64(s32(m_f[FSREG >> 1])).v);
				break;

			default: // unimplemented operation
				m_fcr31 |= FCR31_CE;
				execute_set_input(m_fpu_irq, ASSERT_LINE);
				break;
			}
			break;

		default: // unimplemented operation
			m_fcr31 |= FCR31_CE;
			execute_set_input(m_fpu_irq, ASSERT_LINE);
			break;
		}
		break;
	case 0x31: // LWC1
		load<u32>(SIMMVAL + m_r[RSREG],
			[this, op](u32 data)
		{
			if (FTREG & 1)
				// load the high half of the floating point register
				m_f[FTREG >> 1] = (u64(data) << 32) | u32(m_f[FTREG >> 1]);
			else
				// load the low half of the floating point register
				m_f[FTREG >> 1] = (m_f[FTREG >> 1] & ~0xffffffffULL) | data;
		});
		break;
	case 0x39: // SWC1
		if (FTREG & 1)
			// store the high half of the floating point register
			store<u32>(SIMMVAL + m_r[RSREG], m_f[FTREG >> 1] >> 32);
		else
			// store the low half of the floating point register
			store<u32>(SIMMVAL + m_r[RSREG], m_f[FTREG >> 1]);
		break;
	}
}

template <typename T> void mips1_device_base::set_cop1_reg(unsigned const reg, T const data)
{
	// translate softfloat exception flags to cause register
	if (softfloat_exceptionFlags)
	{
		if (softfloat_exceptionFlags & softfloat_flag_inexact)
			m_fcr31 |= FCR31_CI;
		if (softfloat_exceptionFlags & softfloat_flag_underflow)
			m_fcr31 |= FCR31_CU;
		if (softfloat_exceptionFlags & softfloat_flag_overflow)
			m_fcr31 |= FCR31_CO;
		if (softfloat_exceptionFlags & softfloat_flag_infinite)
			m_fcr31 |= FCR31_CZ;
		if (softfloat_exceptionFlags & softfloat_flag_invalid)
			m_fcr31 |= FCR31_CV;

		// set flags
		m_fcr31 |= ((m_fcr31 & FCR31_CM) >> 10);

		// update exception state
		bool const exception = (m_fcr31 & FCR31_CE) || ((m_fcr31 & FCR31_CM) >> 5) & (m_fcr31 & FCR31_EM);
		execute_set_input(m_fpu_irq, exception ? ASSERT_LINE : CLEAR_LINE);

		if (exception)
			return;
	}

	if (sizeof(T) == 4)
		m_f[reg] = (m_f[reg] & ~0xffffffffULL) | data;
	else
		m_f[reg] = data;
}

mips1core_device_base::translate_result mips1_device_base::translate(int intention, offs_t &address, bool debug)
{
	// check for kernel memory address
	if (BIT(address, 31))
	{
		// check debug or kernel mode
		if (debug || !(SR & SR_KUc))
		{
			switch (address & 0xe0000000)
			{
			case 0x80000000: // kseg0: unmapped, cached, privileged
				address &= ~0xe0000000;
				return m_cache;

			case 0xa0000000: // kseg1: unmapped, uncached, privileged
				address &= ~0xe0000000;
				return UNCACHED;

			case 0xc0000000: // kseg2: mapped, cached, privileged
			case 0xe0000000:
				break;
			}
		}
		else if (SR & SR_KUc)
		{
			address_error(intention, address);

			return ERROR;
		}
	}

	// key is a combination of VPN and ASID
	u32 const key = (address & EH_VPN) | (m_cop0[COP0_EntryHi] & EH_ASID);

	unsigned *mru = m_tlb_mru[intention];

	bool refill = !BIT(address, 31);
	bool modify = false;

	for (unsigned i = 0; i < std::size(m_tlb); i++)
	{
		unsigned const index = mru[i];
		u32 const *const entry = m_tlb[index];

		// test vpn and optionally asid
		u32 const mask = (entry[1] & EL_G) ? EH_VPN : EH_VPN | EH_ASID;
		if ((entry[0] & mask) != (key & mask))
			continue;

		// test valid
		if (!(entry[1] & EL_V))
		{
			refill = false;
			break;
		}

		// test dirty
		if ((intention == TR_WRITE) && !(entry[1] & EL_D))
		{
			refill = false;
			modify = true;
			break;
		}

		// translate the address
		address &= ~EH_VPN;
		address |= (entry[1] & EL_PFN);

		// promote the entry in the mru index
		if (i > 0)
			std::swap(mru[i - 1], mru[i]);

		return (entry[1] & EL_N) ? UNCACHED : m_cache;
	}

	if (!machine().side_effects_disabled() && !debug)
	{
		if (VERBOSE & LOG_TLB)
		{
			if (modify)
				LOGMASKED(LOG_TLB, "asid %2d tlb modify address 0x%08x (%s)\n",
					(m_cop0[COP0_EntryHi] & EH_ASID) >> 6, address, machine().describe_context());
			else
				LOGMASKED(LOG_TLB, "asid %2d tlb miss %c address 0x%08x (%s)\n",
					(m_cop0[COP0_EntryHi] & EH_ASID) >> 6, (intention == TR_WRITE) ? 'w' : 'r', address, machine().describe_context());
		}

		// load tlb exception registers
		m_cop0[COP0_BadVAddr] = address;
		m_cop0[COP0_EntryHi] = key;
		m_cop0[COP0_Context] = (m_cop0[COP0_Context] & PTE_BASE) | ((address >> 10) & BAD_VPN);

		generate_exception(modify ? EXCEPTION_TLBMOD : (intention == TR_WRITE) ? EXCEPTION_TLBSTORE : EXCEPTION_TLBLOAD, refill);
	}

	return ERROR;
}
