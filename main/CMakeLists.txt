# See the build system documentation in IDF programming guide
# for more information about component CMakeLists.txt files.

idf_component_register(
    SRCS
        "main.c"
        "ringbuf.c"
        "adc_sampler.c"
        "ledcontroller.c"       # must match actual filename on disk
        "monitortask.c"         # must match actual filename on disk
    INCLUDE_DIRS "."
)
