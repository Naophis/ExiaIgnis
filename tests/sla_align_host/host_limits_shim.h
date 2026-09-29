// ホストで gen_code_mpc をビルドするための語長の辻褄合わせ(テスト専用)。
// 生成コードは 32bit の long を前提に #error で止めるが、long は使っていない。
#pragma once
#include <climits>
#undef ULONG_MAX
#undef LONG_MAX
#define ULONG_MAX 0xFFFFFFFFUL
#define LONG_MAX 0x7FFFFFFFL
