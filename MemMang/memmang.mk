#
# include file for memmang directory
#
memmang_dir := MemMang
memmang_srcs := $(addprefix $(memmang_dir)/,heap_4.c)
memmang_objs := $(subst .c,.o,$(memmang_srcs))

sources += $(memmang_srcs)
