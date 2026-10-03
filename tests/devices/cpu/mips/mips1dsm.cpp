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

std::pair<std::string, offs_t> disassemble(u32 opcode, bool r3900)
{
	opcode_buffer const buffer(opcode);
	mips1_disassembler disassembler(r3900);
	std::ostringstream output;
	offs_t const result = disassembler.disassemble(output, 0, buffer, buffer);
	return std::make_pair(output.str(), result);
}

} // anonymous namespace


TEST_CASE("MIPS-I CACHE disassembly is R3900-specific", "[devices][cpu][mips]")
{
	constexpr u32 CACHE = 0xbc05'0000; // cache 5,0(zero)
	CHECK(disassemble(CACHE, false).first == ".word  0xbc050000 /*invalid*/");
	CHECK(disassemble(CACHE, true).first == "cache  0x5,0(0)");
}

TEST_CASE("R3900 DERET debugger flags include its delay slot", "[devices][cpu][mips]")
{
	auto const [text, result] = disassemble(0x4200'001f, true);
	CHECK(text == "deret");
	CHECK((result & util::disasm_interface::STEP_OUT) != 0);
	CHECK((result & util::disasm_interface::OVERINSTMASK)
			== util::disasm_interface::step_over_extra(1));
}

TEST_CASE("R3900 RFE disassembly requires the canonical encoding", "[devices][cpu][mips]")
{
	CHECK(disassemble(0x4200'0010, true).first == "rfe");
	CHECK(disassemble(0x4200'0030, true).first == "cop0  0x0000030");
}
