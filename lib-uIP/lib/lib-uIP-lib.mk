#
# include file for uIP directory
#
uIP_lib_dir := lib-uIP/lib
uIP_lib_srcs := $(addprefix $(uIP_lib_dir)/,random.c memb.c list.c)
uIP_lib_objs := $(subst .c,.o,$(uIP_lib_srcs))

sources += $(uIP_lib_srcs)
