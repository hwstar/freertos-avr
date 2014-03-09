#
# include file for uIP/sys directory
#
uIP_sys_dir := lib-uIP/sys
uIP_sys_srcs := $(addprefix $(uIP_sys_dir)/,stimer.c ttimer.c)
uIP_sys_objs := $(subst .c,.o,$(uIP_sys_srcs))

sources += $(uIP_sys_srcs)
