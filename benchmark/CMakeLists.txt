include(FetchContent)
FetchContent_GetProperties(asam_examples)

add_executable(cpm_benchmarks cpm_benchmarks.cpp)

target_link_libraries(cpm_benchmarks PRIVATE Strada::strada benchmark::benchmark_main)

target_compile_definitions(cpm_benchmarks PRIVATE "STRADA_BENCHMARK_DATA_DIR=\"${asam_examples_SOURCE_DIR}\"")

strada_add_warnings(cpm_benchmarks)
