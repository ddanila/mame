// license:BSD-3-Clause
// copyright-holders:Danila Sukharev
/***************************************************************************

    Loopback UDP network bridge

    Carries one raw Ethernet frame per UDP datagram.  It is intended for
    deterministic local peers and test harnesses that cannot create a TAP
    interface.  The bridge always binds to loopback.

    MAME_UDP_NET_LOCAL_PORT   MAME receive port (default 58100)
    MAME_UDP_NET_REMOTE_PORT  peer receive port (default 58101)

***************************************************************************/

#include "netdev_module.h"

#include "modules/osdmodule.h"

#include "asio.h"
#include "netdev_common.h"
#include "osdcore.h"

#include <array>
#include <cerrno>
#include <cstdlib>
#include <limits>
#include <memory>
#include <string>
#include <system_error>
#include <vector>


namespace osd {

namespace {

constexpr u16 DEFAULT_LOCAL_PORT = 58100;
constexpr u16 DEFAULT_REMOTE_PORT = 58101;

u16 environment_port(char const *name, u16 fallback)
{
	char const *const text = osd_getenv(name);
	if (!text || !*text)
		return fallback;

	char *end = nullptr;
	errno = 0;
	unsigned long const value = std::strtoul(text, &end, 10);
	if (errno || !end || *end || !value
			|| (value > std::numeric_limits<u16>::max()))
	{
		osd_printf_warning("%s=%s is not a valid UDP port; using %u\n",
				name, text, fallback);
		return fallback;
	}
	return u16(value);
}


class netdev_udp : public network_device_base
{
public:
	netdev_udp(u16 local_port, u16 remote_port, network_handler &handler)
		: network_device_base(handler)
		, m_socket(m_context)
		, m_remote(asio::ip::address_v4::loopback(), remote_port)
	{
		std::error_code error;
		m_socket.open(asio::ip::udp::v4(), error);
		if (!error)
			m_socket.bind(
					asio::ip::udp::endpoint(
							asio::ip::address_v4::loopback(),
							local_port),
					error);
		if (!error)
			m_socket.non_blocking(true, error);

		if (error)
		{
			osd_printf_error(
					"UDP network bridge could not bind 127.0.0.1:%u: %s\n",
					local_port,
					error.message());
			m_socket.close();
		}
	}

	virtual int send(void const *buf, int len) override
	{
		if (!m_socket.is_open() || (len <= 0))
			return 0;

		std::error_code error;
		std::size_t const sent = m_socket.send_to(
				asio::buffer(buf, std::size_t(len)),
				m_remote,
				0,
				error);
		return error ? 0 : int(sent);
	}

protected:
	virtual int recv_dev(u8 **buf) override
	{
		if (!m_socket.is_open())
			return 0;

		asio::ip::udp::endpoint sender;
		std::error_code error;
		std::size_t const received = m_socket.receive_from(
				asio::buffer(m_buffer),
				sender,
				0,
				error);
		if (error)
		{
			if ((error != asio::error::would_block)
					&& (error != asio::error::try_again))
			{
				osd_printf_verbose(
						"UDP network bridge receive failed: %s\n",
						error.message());
			}
			return 0;
		}

		*buf = m_buffer.data();
		return int(received);
	}

private:
	asio::io_context m_context;
	asio::ip::udp::socket m_socket;
	asio::ip::udp::endpoint m_remote;
	std::array<u8, 2048> m_buffer{};
};


class udp_module : public osd_module, public netdev_module
{
public:
	udp_module()
		: osd_module(OSD_NETDEV_PROVIDER, "udp")
	{
	}

	virtual int init(osd_interface &osd, osd_options const &options) override
	{
		m_local_port = environment_port(
				"MAME_UDP_NET_LOCAL_PORT",
				DEFAULT_LOCAL_PORT);
		m_remote_port = environment_port(
				"MAME_UDP_NET_REMOTE_PORT",
				DEFAULT_REMOTE_PORT);
		m_description = "UDP loopback bridge (127.0.0.1:"
				+ std::to_string(m_local_port)
				+ " -> 127.0.0.1:"
				+ std::to_string(m_remote_port)
				+ ")";
		return 0;
	}

	virtual std::unique_ptr<network_device> open_device(
			int id,
			network_handler &handler) override
	{
		if (id != 0)
			return nullptr;
		return std::make_unique<netdev_udp>(
				m_local_port,
				m_remote_port,
				handler);
	}

	virtual std::vector<network_device_info> list_devices() override
	{
		return { network_device_info{ 0, m_description } };
	}

private:
	u16 m_local_port = DEFAULT_LOCAL_PORT;
	u16 m_remote_port = DEFAULT_REMOTE_PORT;
	std::string m_description;
};

} // anonymous namespace

} // namespace osd


MODULE_DEFINITION(NETDEV_UDP, osd::udp_module)
