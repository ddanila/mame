// license:BSD-3-Clause
// copyright-holders:Danila Sukharev

#include "emu.h"
#include "cpu/mips/mips1dsm.h"

#include "catch.hpp"

#include <sstream>


namespace {

class opcode_buffer : public util::disasm_interface::data_buffer
{
public:
	explicit opcode_buffer(u32 opcode) : m_opcode(opcode) { }

	virtual u8 r8(offs_t) const override { return u8(m_opcode >> 24); }
	virtual u16 r16(offs_t) const override { return u16(m_opcode >> 16); }
	virtual u32 r32(offs_t) const override { return m_opcode; }
	virtual u64 r64(offs_t) const override { return u64(m_opcode) << 32; }

private:
	u32 const m_opcode;
};

std::string disassemble_cache(bool r3900)
{
	constexpr u32 CACHE = 0xbc05'0000; // cache 5,0(zero)
	opcode_buffer const buffer(CACHE);
	mips1_disassembler disassembler(r3900);
	std::ostringstream output;
	disassembler.disassemble(output, 0, buffer, buffer);
	return output.str();
}

} // anonymous namespace


TEST_CASE("MIPS-I CACHE disassembly is R3900-specific", "[devices][cpu][mips]")
{
	CHECK(disassemble_cache(false) == ".word  0xbc050000 /*invalid*/");
	CHECK(disassemble_cache(true) == "cache  0x5,0(0)");
}
