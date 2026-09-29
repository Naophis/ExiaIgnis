extern "C" {
volatile int g_result[64];
volatile int g_mark;
void bench_begin(int id) { g_mark = id; }
void bench_end(int id) { g_mark = -id; }
}
