#
# include file for hd44780 directory
#
hd44780_dir := lib_hd44780
hd44780_srcs := $(addprefix $(hd44780_dir)/,hd44780.c)
hd44780_objs := $(subst .c,.o,$(hd44780_srcs))

sources += $(hd44780_srcs)
