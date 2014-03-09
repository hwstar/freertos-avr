#
# include file for iinchip directory
#
iinchip_dir := lib_iinchip
iinchip_srcs := $(addprefix $(iinchip_dir)/,socket.c socket_util.c w5200.c)
iinchip_objs := $(subst .c,.o,$(iinchip_srcs))

sources += $(iinchip_srcs)
