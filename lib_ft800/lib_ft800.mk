#
# include file for ft800 directory
#
ft800_dir := lib_ft800
ft800_srcs := $(addprefix $(ft800_dir)/,FT_API.c  FT_CoPro_Cmds.c  FT_Gpu_Hal.c  FT_X11_RGB.c)
ft800_objs := $(subst .c,.o,$(ft800_srcs))

sources += $(ft800_srcs)
