#
# include file for lib_io directory
#
lib_io_dir := lib_io
lib_io_srcs := $(addprefix $(lib_io_dir)/,digitalAnalog.c i2cMultiMaster.c serial.c servoPWM.c spi.c )
lib_io_objs := $(subst .c,.o,$(lib_io_srcs))

sources += $(lib_io_srcs)
