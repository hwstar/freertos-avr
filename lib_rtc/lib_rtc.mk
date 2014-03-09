#
# include file for rtc directory
#
rtc_dir := lib_rtc
rtc_srcs := $(addprefix $(rtc_dir)/,rtc.c)
rtc_objs := $(subst .c,.o,$(rtc_srcs))

sources += $(rtc_srcs)
