#
# include file for uIP directory
#
uIP_dir := lib-uIP
uIP_srcs := $(addprefix $(uIP_dir)/,network.c uip6.c uip-ds6.c \
uip-icmp6.c uip-neighbor.c psock.c uip-arp.c uip-ds6-route.c uiplib.c \
uip-split.c slipdev.c uip.c uip-fw.c uip-nd6.c)
uIP_objs := $(subst .c,.o,$(uIP_srcs))

sources += $(uIP_srcs)

include lib-uIP/lib/lib-uIP-lib.mk
include lib-uIP/sys/lib-uIP-sys.mk
