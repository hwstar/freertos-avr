#
# include file for portable directory
#
portable_dir := portable
portable_srcs := $(addprefix $(portable_dir)/,port.c)
portable_objs := $(subst .c,.o,$(portable_srcs))

sources += $(portable_srcs)
