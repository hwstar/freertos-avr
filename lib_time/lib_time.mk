#
# include file for lib_time directory
#
lib_time_dir := lib_time
lib_time_srcs := $(addprefix $(lib_time_dir)/,asc_store.c gm_sidereal.c mk_gmtime.c strftime.c \
asctime.c gmtime.c mktime.c sun_rise.c asctime_r.c gmtime_r.c month_length.c sun_set.c \
ctime.c isLeap.c moon_phase.c system_time.c \
ctime_r.c isotime.c print_lz.c time.c \
daylight_seconds.c isotime_r.c set_dst.c tm_store.c \
difftime.c iso_week_date.c set_position.c utc_offset.c \
dst_pointer.c iso_week_date_r.c set_system_time.c week_of_month.c \
equation_of_time.c lm_sidereal.c set_zone.c week_of_year.c \
fatfs_time.c localtime.c solar_declination.c \
geo_location.c localtime_r.c solar_noon.c, system_tick.S)
lib_time_objs := $(subst .c,.o,$(lib_time_srcs))

sources += $(lib_time_srcs)
